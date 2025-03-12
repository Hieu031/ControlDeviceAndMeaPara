# ControlDeviceAndMeaPara
a project used ESP32 communication with CoreIOT via mqtt to show temperature and humidity, Control led and Servo at Dashboard.

紹介: プロジェクトにおいて使用した主なプロトコルは WiFi, MQTT. 
１．プロジェクトを説明する
―　温度と湿度を読み取ってCoreIOTプラットフォームに情報を送信します。
―　温度に応じてポンプを自動的にオン／オフすることです。そして、温度が閾値を越えると、警告ブザーが鳴ります。
―　LEDとかサーボとかポンプをウエブサイトで制御します。
２．主なコンポーネント
―　ESP32マイクロコントローラー
―　DHT22センサー
―　Relay, LED, ブザー、他の部品
3. 使用したツール
―　Visual Studio Code、platformIO
―　CoreIOT
―　Wokwi
