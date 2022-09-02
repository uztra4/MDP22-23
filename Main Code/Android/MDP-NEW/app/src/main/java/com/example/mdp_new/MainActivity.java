package com.example.mdp_new;


import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;
import android.widget.ImageButton;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.home);
        Button comms_button = findViewById(R.id.button1);
        Button bluetooth_button = findViewById(R.id.button2);
        Button arena_button = findViewById(R.id.button3);

        comms_button.setOnClickListener(v -> openCommsView());
        bluetooth_button.setOnClickListener(v -> openBluetoothView());
        arena_button.setOnClickListener(v -> openArenaView());
    }


    public void openCommsView() {
        Intent intent = new Intent(this, Communication.class);
        startActivity(intent);
    }

    public void openBluetoothView() {
        Intent intent = new Intent(this, Bluetooth.class);
        startActivity(intent);
    }

    public void openArenaView() {
        Intent intent = new Intent(this, Arena.class);
        startActivity(intent);
    }
}