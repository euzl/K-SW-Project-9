package com.robpercival.memorableplaces;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.location.Location;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.concurrent.TimeUnit;

import static android.content.ContentValues.TAG;


public class PopupActivity extends Activity {
    TextView txtText;

    Location loc;
    double latitude, longitude;

    final int REQUEST_ENABLE_BT = 4;
    protected void setLocation(Location location){
        latitude = location.getLatitude();
        longitude = location.getLongitude();
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //타이틀바 없애기
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.popup_activity);

        //UI 객체생성
        txtText = (TextView)findViewById(R.id.txtText);

        //데이터 가져오기
        Intent intent = getIntent();
        String data = intent.getStringExtra("data");

        StringTokenizer st = new StringTokenizer(data);
        latitude = Double.valueOf(st.nextToken());
        longitude = Double.valueOf(st.nextToken());

        txtText.setText(data);
    }

    //확인 버튼 클릭
    public void mOnClose(View v) throws InterruptedException {
      //  데이터 전달하기
//        Intent intent = new Intent();
//        intent.putExtra("result", "Close Popup");
//        setResult(RESULT_OK, intent);

        //액티비티(팝업) 닫기
        BluetoothSerialClient bsc = BluetoothSerialClient.getInstance();
        if(bsc == null) {
            Toast.makeText(getApplicationContext(), "블루투스를 사용할 수 없는 기기입니다.", Toast.LENGTH_LONG).show();
        }
        if(!bsc.isEnabled()) {
            //enabling Bluetooth
            //print asking to user about enabling Bluetooth
            //if Bluetooth is enabled already, ignore this code
            bsc.enableBluetooth(PopupActivity.this,
                    new BluetoothSerialClient.OnBluetoothEnabledListener() {
                        @Override
                        public void onBluetoothEnabled(boolean success) {
                            //if success == false, user don't want Bluetooth
                        }
                    });
        }

        final ArrayList<BluetoothDevice> deviceList = new ArrayList<BluetoothDevice>();
        Set<BluetoothDevice> pairedDevices = bsc.getPairedDevices();
        deviceList.clear();
        deviceList.addAll(pairedDevices);

        //scan nearby devices
        bsc.scanDevices(this, new BluetoothSerialClient.OnScanListener() {
            @Override
            public void onStart() {
                //scan start
            }

            @Override
            public void onFoundDevice(BluetoothDevice bluetoothDevice) {
                if(deviceList.contains(bluetoothDevice)) {
                    deviceList.remove(bluetoothDevice);
                }
                deviceList.add(bluetoothDevice);
            }

            @Override
            public void onFinish() {
                //scan finished
            }
        });

        BluetoothSerialClient.BluetoothStreamingHandler handler
                                    = new BluetoothSerialClient.BluetoothStreamingHandler() {
            @Override
            public void onError(Exception e) {
                //
                System.out.print("Data communication Error!");
            }

            @Override
            public void onConnected() {

            }

            @Override
            public void onDisconnected() {

            }

            @Override
            public void onData(byte[] buffer, int length) {

            }
        };
        for(BluetoothDevice bd : deviceList){
            if(bd.getAddress().equals("B8:27:EB:E0:23:74")){
                if(!bsc.connect(this, bd, handler)){
                    Log.println(Log.ERROR, "bluetooth connect", "bluetooth connection failed");
                    SystemClock.sleep(100);
                }
                else {
                    Log.println(Log.ERROR, "bluetooth connect", "bluetooth connection succeed");
                    txtText.setText("blue tooth connected!");
                    try{ Thread.sleep(3000);}catch(Exception e) {}
                    handler.write(String.valueOf(latitude).getBytes());
                    //handler.write("\n".getBytes())
                    SystemClock.sleep(3000);
                    try{ Thread.sleep(500);}catch(Exception e) {}
                    handler.write(String.valueOf(longitude).getBytes());
                    SystemClock.sleep(3000);
                    bsc.claer();
                }
            }

        }

        finish();
    }

    public boolean onTouchEvent(MotionEvent event) {
        //바깥레이어 클릭시 안닫히게
        if(event.getAction()==MotionEvent.ACTION_OUTSIDE){
            return false;
        }
        return true;
    }

}
