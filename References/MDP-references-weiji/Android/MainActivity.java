package com.example.mdp_test;

import androidx.appcompat.app.AppCompatActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;

import com.google.android.material.snackbar.Snackbar;
import com.example.mdp_test.BluetoothConnectionService;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity{
    private static final String TAG = "MainActivity";
    private static final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");;

    BluetoothDevice mBTDevice;
    Button btnSend;
    TextView incomingMessages;
    StringBuilder messages;
    EditText etSend;

    private BluetoothConnectionService mBluetoothConnection;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        btnSend = (Button) findViewById(R.id.btnSend);
        Button btnONOFF = (Button) findViewById(R.id.btnONOFF);

        incomingMessages = (TextView) findViewById(R.id.tvReceive);
        etSend = (EditText) findViewById(R.id.etSend);

        messages = new StringBuilder();

        btnONOFF.setOnClickListener(view -> {
            Intent BTintent = new Intent(this, BluetoothActivity.class);
            startActivity(BTintent);
        });

        btnSend.setOnClickListener(view -> {
            if(mBluetoothConnection != null){
                byte[] bytes = etSend.getText().toString().getBytes(Charset.defaultCharset());
                mBluetoothConnection.write(bytes);
                etSend.setText("");
            }
            //mBluetoothConnection = new BluetoothConnectionService(MainActivity.this);
        });
    }
}