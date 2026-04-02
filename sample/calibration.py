#!/usr/bin/env python3
import rospy
import sys
from umoru_arm.srv import SensorCalib

def call_calibration():
    rospy.init_node('calib_client_node', disable_signals=False) # ROSのシグナルハンドラを有効化

    # パラメータの取得
    try:
        rarm_ids = rospy.get_param('/rarm/air_board_ids', [])
        larm_ids = rospy.get_param('/larm/air_board_ids', [])
        rospy.loginfo(f"有効なIDリスト - rarm: {rarm_ids}, larm: {larm_ids}")
    except KeyError as e:
        rospy.logerr(f"パラメータが見つかりません: {e}")
        return

    # サービスの待機
    rospy.loginfo("各サービスを待機中...")
    try:
        rospy.wait_for_service('/rarm/calibrate_sensor', timeout=2.0)
        rospy.wait_for_service('/larm/calibrate_sensor', timeout=2.0)
    except rospy.ROSException:
        rospy.logwarn("一部のサービスが見つかりませんが、続行します。")

    rarm_service = rospy.ServiceProxy('/rarm/calibrate_sensor', SensorCalib)
    larm_service = rospy.ServiceProxy('/larm/calibrate_sensor', SensorCalib)

    # --- メインループ ---
    try:
        while not rospy.is_shutdown():
            print("\n" + "="*40)
            print("【キャリブレーション・ツール】 終了するには Ctrl+D を押してください")
            
            # ID入力
            user_input = input("校正するセンサーIDを入力: ").strip()
            if not user_input:
                continue
            idx = int(user_input)

            # 腕の判定
            if idx in rarm_ids:
                target_service = rarm_service
                arm_label = "右腕 (rarm)"
            elif idx in larm_ids:
                target_service = larm_service
                arm_label = "左腕 (larm)"
            else:
                print(f"❌ Error: ID {idx} はリストに存在しません。")
                continue

            print(f"▶ {arm_label} センサー {idx} を選択中")

            # ステップ1: 開始
            input("　[Step 1] Enterで【空気供給を開始】...")
            resp_start = target_service(idx=idx, start=True)
            if resp_start.success:
                print(f"　>> 供給中... (バルブが開きました)")

            # ステップ2: 停止・計測
            input("　[Step 2] Enterで【停止して最大圧力を記録】...")
            resp_stop = target_service(idx=idx, start=False)
            
            if resp_stop.success:
                print(f"✅ 完了！ 最大圧力: {resp_stop.max_pressure:.4f}")
            else:
                print("❌ 校正に失敗しました。")

    except (KeyboardInterrupt, EOFError):
        # Ctrl+C または Ctrl+D を検知
        print("\n\n[終了] ユーザーによって中断されました。")
        sys.exit(0)
    except Exception as e:
        print(f"\n予期しないエラーが発生しました: {e}")

if __name__ == "__main__":
    call_calibration()
