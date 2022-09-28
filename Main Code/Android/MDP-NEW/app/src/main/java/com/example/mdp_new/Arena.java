package com.example.mdp_new;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.animation.ObjectAnimator;

import androidx.appcompat.app.AppCompatActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.Map;

public class Arena extends AppCompatActivity {
    private static final int SNAP_GRID_INTERVAL = 40;

    private static final int ANIMATOR_DURATION = 1000;

    private boolean isObstacle1LongClicked = false;
    private boolean isObstacle2LongClicked = false;
    private boolean isObstacle3LongClicked = false;
    private boolean isObstacle4LongClicked = false;
    private boolean isObstacle5LongClicked = false;

    private int sequence = 0;

    Button IRButton, SPButton, resetButton, preset1Button, preset2Button, preset3Button, timerButton;
    TextView statusWindow;  //Added statusWindow to declarations
    ImageView obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8;
    ImageView car;

    TextView car_x, car_y, car_dir;

    Map<Integer, ImageView> obstacles;

    Map<String, String> commands = new HashMap<String, String>() {{
        put("forward", "0100");
        put("reverse", "0101");
        put("turnLeft", "0902");
        put("turnRight", "0903");
    }};

    Map<String, Integer> resources = new HashMap<String, Integer>() {{
        put("o1n", R.drawable.obstacle_1_n);
        put("o1e", R.drawable.obstacle_1_e);
        put("o1s", R.drawable.obstacle_1_s);
        put("o1w", R.drawable.obstacle_1_w);

        put("o2n", R.drawable.obstacle_2_n);
        put("o2e", R.drawable.obstacle_2_e);
        put("o2s", R.drawable.obstacle_2_s);
        put("o2w", R.drawable.obstacle_2_w);

        put("o3n", R.drawable.obstacle_3_n);
        put("o3e", R.drawable.obstacle_3_e);
        put("o3s", R.drawable.obstacle_3_s);
        put("o3w", R.drawable.obstacle_3_w);

        put("o4n", R.drawable.obstacle_4_n);
        put("o4e", R.drawable.obstacle_4_e);
        put("o4s", R.drawable.obstacle_4_s);
        put("o4w", R.drawable.obstacle_4_w);

        put("o5n", R.drawable.obstacle_5_n);
        put("o5e", R.drawable.obstacle_5_e);
        put("o5s", R.drawable.obstacle_5_s);
        put("o5w", R.drawable.obstacle_5_w);

        put("o6n", R.drawable.obstacle_6_n);
        put("o6e", R.drawable.obstacle_6_e);
        put("o6s", R.drawable.obstacle_6_s);
        put("o6w", R.drawable.obstacle_6_w);

        put("o7n", R.drawable.obstacle_7_n);
        put("o7e", R.drawable.obstacle_7_e);
        put("o7s", R.drawable.obstacle_7_s);
        put("o7w", R.drawable.obstacle_7_w);

        put("o8n", R.drawable.obstacle_8_n);
        put("o8e", R.drawable.obstacle_8_e);
        put("o8s", R.drawable.obstacle_8_s);
        put("o8w", R.drawable.obstacle_8_w);

        put("11", R.drawable.number_1);
        put("11n", R.drawable.number_1_n);
        put("11e", R.drawable.number_1_e);
        put("11s",R.drawable.number_1_s);
        put("11w",R.drawable.number_1_w);
        put("12", R.drawable.number_2);
        put("12n", R.drawable.number_2_n);
        put("12e", R.drawable.number_2_e);
        put("12s",R.drawable.number_2_s);
        put("12w",R.drawable.number_2_w);
        put("13", R.drawable.number_3);
        put("13n", R.drawable.number_3_n);
        put("13e", R.drawable.number_3_e);
        put("13s",R.drawable.number_3_s);
        put("13w",R.drawable.number_3_w);
        put("14", R.drawable.number_4);
        put("14n", R.drawable.number_4_n);
        put("14e", R.drawable.number_4_e);
        put("14s",R.drawable.number_4_s);
        put("14w",R.drawable.number_4_w);
        put("15", R.drawable.number_5);
        put("15n", R.drawable.number_5_n);
        put("15e", R.drawable.number_5_e);
        put("15s",R.drawable.number_5_s);
        put("15w",R.drawable.number_5_w);
        put("16", R.drawable.number_6);
        put("16n", R.drawable.number_6_n);
        put("16e", R.drawable.number_6_e);
        put("16s",R.drawable.number_6_s);
        put("16w",R.drawable.number_6_w);
        put("17", R.drawable.number_7);
        put("17n", R.drawable.number_7_n);
        put("17e", R.drawable.number_7_e);
        put("17s",R.drawable.number_7_s);
        put("17w",R.drawable.number_7_w);
        put("18", R.drawable.number_8);
        put("18n", R.drawable.number_8_n);
        put("18e", R.drawable.number_8_e);
        put("18s",R.drawable.number_8_s);
        put("18w",R.drawable.number_8_w);
        put("19", R.drawable.number_9);
        put("19n", R.drawable.number_9_n);
        put("19e", R.drawable.number_9_e);
        put("19s",R.drawable.number_9_s);
        put("19w",R.drawable.number_9_w);
        put("20", R.drawable.alphabet_a);
        put("20n", R.drawable.alphabet_a_n);
        put("20e", R.drawable.alphabet_a_e);
        put("20s",R.drawable.alphabet_a_s);
        put("20w",R.drawable.alphabet_a_w);
        put("21", R.drawable.alphabet_b);
        put("21n", R.drawable.alphabet_b_n);
        put("21e", R.drawable.alphabet_b_e);
        put("21s",R.drawable.alphabet_b_s);
        put("21w",R.drawable.alphabet_b_w);
        put("22", R.drawable.alphabet_c);
        put("22n", R.drawable.alphabet_c_n);
        put("22e", R.drawable.alphabet_c_e);
        put("22s",R.drawable.alphabet_c_s);
        put("22w",R.drawable.alphabet_c_w);
        put("23", R.drawable.alphabet_d);
        put("23n", R.drawable.alphabet_d_n);
        put("23e", R.drawable.alphabet_d_e);
        put("23s",R.drawable.alphabet_d_s);
        put("23w",R.drawable.alphabet_d_w);
        put("24", R.drawable.alphabet_e);
        put("24n", R.drawable.alphabet_e_n);
        put("24e", R.drawable.alphabet_e_e);
        put("24s",R.drawable.alphabet_e_s);
        put("24w",R.drawable.alphabet_e_w);
        put("25", R.drawable.alphabet_f);
        put("25n", R.drawable.alphabet_f_n);
        put("25e", R.drawable.alphabet_f_e);
        put("25s",R.drawable.alphabet_f_s);
        put("25w",R.drawable.alphabet_f_w);
        put("26", R.drawable.alphabet_g);
        put("26n", R.drawable.alphabet_g_n);
        put("26e", R.drawable.alphabet_g_e);
        put("26s",R.drawable.alphabet_g_s);
        put("26w",R.drawable.alphabet_g_w);
        put("27", R.drawable.alphabet_h);
        put("27n", R.drawable.alphabet_h_n);
        put("27e", R.drawable.alphabet_h_e);
        put("27s",R.drawable.alphabet_h_s);
        put("27w",R.drawable.alphabet_h_w);
        put("28", R.drawable.alphabet_s);
        put("28n", R.drawable.alphabet_s_n);
        put("28e", R.drawable.alphabet_s_e);
        put("28s",R.drawable.alphabet_s_s);
        put("28w",R.drawable.alphabet_s_w);
        put("29", R.drawable.alphabet_t);
        put("29n", R.drawable.alphabet_t_n);
        put("29e", R.drawable.alphabet_t_e);
        put("29s",R.drawable.alphabet_t_s);
        put("29w",R.drawable.alphabet_t_w);
        put("30", R.drawable.alphabet_u);
        put("30n", R.drawable.alphabet_u_n);
        put("30e", R.drawable.alphabet_u_e);
        put("30s",R.drawable.alphabet_u_s);
        put("30w",R.drawable.alphabet_u_w);
        put("31", R.drawable.alphabet_v);
        put("31n", R.drawable.alphabet_v_n);
        put("31e", R.drawable.alphabet_v_e);
        put("31s",R.drawable.alphabet_v_s);
        put("31w",R.drawable.alphabet_v_w);
        put("32", R.drawable.alphabet_w);
        put("32n", R.drawable.alphabet_w_n);
        put("32e", R.drawable.alphabet_w_e);
        put("32s",R.drawable.alphabet_w_s);
        put("32w",R.drawable.alphabet_w_w);
        put("33", R.drawable.alphabet_x);
        put("33n", R.drawable.alphabet_x_n);
        put("33e", R.drawable.alphabet_x_e);
        put("33s",R.drawable.alphabet_x_s);
        put("33w",R.drawable.alphabet_x_w);
        put("34", R.drawable.alphabet_y);
        put("34n", R.drawable.alphabet_y_n);
        put("34e", R.drawable.alphabet_y_e);
        put("34s",R.drawable.alphabet_y_s);
        put("34w",R.drawable.alphabet_y_w);
        put("35", R.drawable.alphabet_z);
        put("35n", R.drawable.alphabet_z_n);
        put("35e", R.drawable.alphabet_z_e);
        put("35s",R.drawable.alphabet_z_s);
        put("35w",R.drawable.alphabet_z_w);

        put("36", R.drawable.arrow_up);
        put("37", R.drawable.arrow_down);
        put("39", R.drawable.arrow_left);
        put("38", R.drawable.arrow_right);
        put("41", R.drawable.bullseye);
        put("40", R.drawable.circle);

        put("42", R.drawable.yellow_question_mark);
        put("43", R.drawable.red_question_mark);
    }};

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.arena);
        //messageBox = findViewById(R.id.message_box);
        // Obstacles

        LocalBroadcastManager.getInstance(this).registerReceiver(myReceiver, new IntentFilter("incomingMessage"));
        obstacle1 = findViewById(R.id.obstacle1);

        obstacle2 = findViewById(R.id.obstacle2);
        obstacle3 = findViewById(R.id.obstacle3);
        obstacle4 = findViewById(R.id.obstacle4);
        obstacle5 = findViewById(R.id.obstacle5);
        obstacle6 = findViewById(R.id.obstacle6);
        obstacle7 = findViewById(R.id.obstacle7);
        obstacle8 = findViewById(R.id.obstacle8);

        obstacles = new HashMap<Integer, ImageView>() {{
            put(1, obstacle1);
            put(2, obstacle2);
            put(3, obstacle3);
            put(4, obstacle4);
            put(5, obstacle5);
            put(6, obstacle6);
            put(7, obstacle7);
            put(8, obstacle8);
        }};

        obstacle1.setOnClickListener(view -> {
            obstacle1.setRotation((obstacle1.getRotation() + 90) % 360);
            int orientation = (int) obstacle1.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle1.setImageResource(resources.get("o1n"));
                    break;
                case 1:
                    obstacle1.setImageResource(resources.get("o1e"));
                    break;
                case 2:
                    obstacle1.setImageResource(resources.get("o1s"));
                    break;
                case 3:
                    obstacle1.setImageResource(resources.get("o1w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle2.setOnClickListener(view -> {
            obstacle2.setRotation((obstacle2.getRotation() + 90) % 360);
            int orientation = (int) obstacle2.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle2.setImageResource(resources.get("o2n"));
                    break;
                case 1:
                    obstacle2.setImageResource(resources.get("o2e"));
                    break;
                case 2:
                    obstacle2.setImageResource(resources.get("o2s"));
                    break;
                case 3:
                    obstacle2.setImageResource(resources.get("o2w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle3.setOnClickListener(view -> {
            obstacle3.setRotation((obstacle3.getRotation() + 90) % 360);
            int orientation = (int) obstacle3.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle3.setImageResource(resources.get("o3n"));
                    break;
                case 1:
                    obstacle3.setImageResource(resources.get("o3e"));
                    break;
                case 2:
                    obstacle3.setImageResource(resources.get("o3s"));
                    break;
                case 3:
                    obstacle3.setImageResource(resources.get("o3w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle4.setOnClickListener(view -> {
            obstacle4.setRotation((obstacle4.getRotation() + 90) % 360);
            int orientation = (int) obstacle4.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle4.setImageResource(resources.get("o4n"));
                    break;
                case 1:
                    obstacle4.setImageResource(resources.get("o4e"));
                    break;
                case 2:
                    obstacle4.setImageResource(resources.get("o4s"));
                    break;
                case 3:
                    obstacle4.setImageResource(resources.get("o4w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle5.setOnClickListener(view -> {
            obstacle5.setRotation((obstacle5.getRotation() + 90) % 360);
            int orientation = (int) obstacle5.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle5.setImageResource(resources.get("o5n"));
                    break;
                case 1:
                    obstacle5.setImageResource(resources.get("o5e"));
                    break;
                case 2:
                    obstacle5.setImageResource(resources.get("o5s"));
                    break;
                case 3:
                    obstacle5.setImageResource(resources.get("o5w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle6.setOnClickListener(view -> {
            obstacle6.setRotation((obstacle6.getRotation() + 90) % 360);
            int orientation = (int) obstacle6.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle6.setImageResource(resources.get("o6n"));
                    break;
                case 1:
                    obstacle6.setImageResource(resources.get("o6e"));
                    break;
                case 2:
                    obstacle6.setImageResource(resources.get("o6s"));
                    break;
                case 3:
                    obstacle6.setImageResource(resources.get("o6w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle7.setOnClickListener(view -> {
            obstacle7.setRotation((obstacle7.getRotation() + 90) % 360);
            int orientation = (int) obstacle7.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle7.setImageResource(resources.get("o7n"));
                    break;
                case 1:
                    obstacle7.setImageResource(resources.get("o7e"));
                    break;
                case 2:
                    obstacle7.setImageResource(resources.get("o7s"));
                    break;
                case 3:
                    obstacle7.setImageResource(resources.get("o7w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle8.setOnClickListener(view -> {
            obstacle8.setRotation((obstacle8.getRotation() + 90) % 360);
            int orientation = (int) obstacle8.getRotation();
            switch (((orientation / 90) % 4 + 4) % 4) {
                case 0:
                    obstacle8.setImageResource(resources.get("o8n"));
                    break;
                case 1:
                    obstacle8.setImageResource(resources.get("o8e"));
                    break;
                case 2:
                    obstacle8.setImageResource(resources.get("o8s"));
                    break;
                case 3:
                    obstacle8.setImageResource(resources.get("o8w"));
                    break;
                default:
                    // Shouldn't reach this case
                    break;
            }
        });

        obstacle1.setOnLongClickListener(view -> {
            isObstacle1LongClicked = true;
            return false;
        });

        obstacle2.setOnLongClickListener(view -> {
            isObstacle2LongClicked = true;
            return false;
        });

        obstacle3.setOnLongClickListener(view -> {
            isObstacle3LongClicked = true;
            return false;
        });

        obstacle4.setOnLongClickListener(view -> {
            isObstacle4LongClicked = true;
            return false;
        });

        obstacle5.setOnLongClickListener(view -> {
            isObstacle5LongClicked = true;
            return false;
        });

        obstacle6.setOnLongClickListener(view -> {
            isObstacle5LongClicked = true;
            return false;
        });

        obstacle7.setOnLongClickListener(view -> {
            isObstacle5LongClicked = true;
            return false;
        });

        obstacle8.setOnLongClickListener(view -> {
            isObstacle5LongClicked = true;
            return false;
        });

        obstacle1.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle1LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle1.setX(obstacle1.getX() + dx);
                        obstacle1.setY(obstacle1.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle1.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle1.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        Log.d("current location","obstacle is at "+ snapToX + "," +snapToY);
                        obstacle1.setX(snapToX);
                        obstacle1.setY(snapToY);
                        isObstacle1LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 1 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle2.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle2LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle2.setX(obstacle2.getX() + dx);
                        obstacle2.setY(obstacle2.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle2.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle2.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle2.setX(snapToX);
                        obstacle2.setY(snapToY);
                        isObstacle2LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 2 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle3.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle3LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle3.setX(obstacle3.getX() + dx);
                        obstacle3.setY(obstacle3.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle3.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle3.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle3.setX(snapToX);
                        obstacle3.setY(snapToY);
                        isObstacle3LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 3 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle4.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle4LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle4.setX(obstacle4.getX() + dx);
                        obstacle4.setY(obstacle4.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle4.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle4.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle4.setX(snapToX);
                        obstacle4.setY(snapToY);
                        isObstacle4LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 4 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle5.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle5LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle5.setX(obstacle5.getX() + dx);
                        obstacle5.setY(obstacle5.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle5.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle5.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle5.setX(snapToX);
                        obstacle5.setY(snapToY);
                        isObstacle5LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 5 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle6.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle5LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle6.setX(obstacle6.getX() + dx);
                        obstacle6.setY(obstacle6.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle6.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle6.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle6.setX(snapToX);
                        obstacle6.setY(snapToY);
                        isObstacle5LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 6 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle7.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle5LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle7.setX(obstacle7.getX() + dx);
                        obstacle7.setY(obstacle7.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle7.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle7.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle7.setX(snapToX);
                        obstacle7.setY(snapToY);
                        isObstacle5LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 7 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        obstacle8.setOnTouchListener(new View.OnTouchListener() {
            int x = 0;
            int y = 0;
            int dx = 0;
            int dy = 0;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (!isObstacle5LongClicked) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        x = (int) event.getX();
                        y = (int) event.getY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        dx = (int) event.getX() - x;
                        dy = (int) event.getY() - y;

                        obstacle8.setX(obstacle8.getX() + dx);
                        obstacle8.setY(obstacle8.getY() + dy);
                        break;
                    case MotionEvent.ACTION_UP:
                        int snapToX = ((int) ((obstacle8.getX() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        int snapToY = ((int) ((obstacle8.getY() + SNAP_GRID_INTERVAL / 2) / SNAP_GRID_INTERVAL)) * SNAP_GRID_INTERVAL;
                        obstacle8.setX(snapToX);
                        obstacle8.setY(snapToY);
                        isObstacle5LongClicked = false;
                        // Bluetooth message
                        if (BluetoothConnectionService.BluetoothConnectionStatus) {
                            byte[] bytes = String.format("Obstacle 8 moved to %d, %d",snapToX/40,snapToY/40).getBytes(Charset.defaultCharset());
                            BluetoothConnectionService.write(bytes);
                        }
                        break;
                    default:
                        break;
                }
                return false;
            }
        });

        // Declarations
        IRButton = findViewById(R.id.IRButton);
        SPButton = findViewById(R.id.SPBtn);
        resetButton = findViewById(R.id.resetButton);
        car = findViewById(R.id.car);
        car_x = findViewById(R.id.x_tv);
        car_y = findViewById(R.id.y_tv);
        car_dir = findViewById(R.id.dir_tv);
        preset1Button = findViewById(R.id.preset1Button);
        preset2Button = findViewById(R.id.preset2Button);
        preset3Button = findViewById(R.id.preset3Button);
        timerButton = findViewById(R.id.timerButton);
        statusWindow = findViewById(R.id.statusWindowText);

        // Events
        IRButton.setOnClickListener(view -> sendObstaclesEvent());
        SPButton.setOnClickListener(view-> beginSPTask());
        resetButton.setOnClickListener(view -> resetObstaclesButton());
        preset1Button.setOnClickListener(view -> setPreset1Button());
        preset2Button.setOnClickListener(view -> setPreset2Button());
        preset3Button.setOnClickListener(view -> setPreset3Button());
        timerButton.setOnClickListener(view -> stopTimerButton());

        // Initialize car to bottom left
        car.setX(1 * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL);
        car.setY(18 * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL);
        updateXYDirText();


        // Movement Buttons
        ImageButton forwardButton = (ImageButton) findViewById(R.id.forwardButton);
        forwardButton.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v) {
                Log.d("COMMS DEBUG", "forward");

                // Bluetooth message
                if (BluetoothConnectionService.BluetoothConnectionStatus) {
                    byte[] bytes = "STM:w10n".getBytes(Charset.defaultCharset());
                    BluetoothConnectionService.write(bytes);
                }

                // Animation
                forwardButtonCommand();
            }
        });

        ImageButton reverseButton = (ImageButton) findViewById(R.id.reverseButton);
        reverseButton.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                Log.d("COMMS DEBUG","reverse");

                // Bluetooth message
                if (BluetoothConnectionService.BluetoothConnectionStatus) {
                    byte[] bytes = "STM:s10n".getBytes(Charset.defaultCharset());
                    BluetoothConnectionService.write(bytes);
                }

                // Animation
                reverseButtonCommand();
            }
        });

        ImageButton leftButton = (ImageButton) findViewById(R.id.leftButton);
        leftButton.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                Log.d("COMMS DEBUG","left");
                if (BluetoothConnectionService.BluetoothConnectionStatus) {
                    byte[] bytes = "STM:a".getBytes(Charset.defaultCharset());
                    BluetoothConnectionService.write(bytes);
                }

                leftButtonCommand();
            }
        });

        ImageButton rightButton = (ImageButton) findViewById(R.id.rightButton);
        rightButton.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v) {
                Log.d("COMMS DEBUG", "right");
                if (BluetoothConnectionService.BluetoothConnectionStatus) {
                    byte[] bytes = "STM:d".getBytes(Charset.defaultCharset());
                    BluetoothConnectionService.write(bytes);
                }

                rightButtonCommand();
            }
        });
    }

    // MOVEMENT COMMANDS
    private void forwardButtonCommand() {
        int orientation = (int) car.getRotation();
        int new_x, new_y;
        ObjectAnimator animator;
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0: //North
                new_y = (int) car.getY() - SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 1:
                new_x = (int) car.getX() + SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 2:
                new_y = (int) car.getY() + SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 3:
                new_x = (int) car.getX() - SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            default:
                // Shouldn't reach this case
                break;
        }
    }


    private void reverseButtonCommand() {
        int orientation = (int) car.getRotation();
        int new_x, new_y;
        ObjectAnimator animator;
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                new_y = (int) car.getY() + SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 1:
                new_x = (int) car.getX() - SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 2:
                new_y = (int) car.getY() - SNAP_GRID_INTERVAL;
                car.setY(new_y);
                animator = ObjectAnimator.ofFloat(car, "y", new_y);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            case 3:
                new_x = (int) car.getX() + SNAP_GRID_INTERVAL;
                car.setX(new_x);
                animator = ObjectAnimator.ofFloat(car, "x", new_x);
                animator.setDuration(ANIMATOR_DURATION);
                animator.start();
                updateXYDirText();
                break;
            default:
                // Shouldn't reach this case
                break;
        }
    }

    public void leftButtonCommand(){
        int orientation = (int) car.getRotation();
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                car.setRotation(270);
                break;
            case 1:
                car.setRotation(0);
                break;
            case 2:
                car.setRotation(90);
                break;
            case 3:
                car.setRotation(180);
                break;
            default:
                // Shouldn't reach this case
                break;
        }

        updateXYDirText();
    };

    private void rightButtonCommand(){
        int orientation = (int) car.getRotation();
        switch (((orientation / 90) % 4 + 4) % 4) {
            case 0:
                car.setRotation(90);
                break;
            case 1:
                car.setRotation(180);
                break;
            case 2:
                car.setRotation(270);
                break;
            case 3:
                car.setRotation(0);
                break;
            default:
                // Shouldn't reach this case
                break;
        }

        updateXYDirText();
    }

    private void setObstacleImage(int obstacleNumber, String image) {
        int orientation = (int) obstacles.get(obstacleNumber).getRotation();

        if(orientation==0) {
            obstacles.get(obstacleNumber).setImageResource(resources.get(image+"n"));
        }
        else if(orientation==90) {
            obstacles.get(obstacleNumber).setImageResource(resources.get(image+"e"));
        }
        else if(orientation==180) {
            obstacles.get(obstacleNumber).setImageResource(resources.get(image+"s"));
        }
        else if(orientation==270) {
            obstacles.get(obstacleNumber).setImageResource(resources.get(image+"w"));
        }
        else{
            obstacles.get(obstacleNumber).setImageResource(resources.get(image));
            obstacles.get(obstacleNumber).setRotation(0);
        }
        }


    TextView messageBox;

    private void stopTimerButton() {
        Chronometer IRTimer = (Chronometer) findViewById(R.id.IRTimer);
        IRTimer.stop();
        updateStatusWindow("Ready To Start");
//        updateStatusWindow("Image Recognition Stopped");
    }

    private void sendObstaclesEvent() {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder
                .append("ALG:")
                .append(getObstacleString(obstacle1)+"0;")
                .append(getObstacleString(obstacle2)+"1;")
                .append(getObstacleString(obstacle3)+"2;")
                .append(getObstacleString(obstacle4)+"3;")
                .append(getObstacleString(obstacle5)+"4;")
                .append(getObstacleString(obstacle6)+"5;")
                .append(getObstacleString(obstacle7)+"6;")
                .append(getObstacleString(obstacle8)+"7;");

        if (BluetoothConnectionService.BluetoothConnectionStatus == true) {
            //Toast.makeText(this, stringBuilder.toString(), Toast.LENGTH_LONG).show();
            byte[] bytes = stringBuilder.toString().getBytes(Charset.defaultCharset());
            BluetoothConnectionService.write(bytes);
            Toast.makeText(Arena.this, "Image Recognition Started.", Toast.LENGTH_LONG).show();
            updateStatusWindow("IR started");

        } else {
            Toast.makeText(Arena.this, "Please connect to Bluetooth.", Toast.LENGTH_LONG).show();
        }

        Chronometer IRTimer = (Chronometer) findViewById(R.id.IRTimer);
        long elapsedRealtime = SystemClock.elapsedRealtime();
        IRTimer.setBase(elapsedRealtime);
        IRTimer.start();
    }

    private void beginSPTask() {
        if (BluetoothConnectionService.BluetoothConnectionStatus == true) {
            byte[] bytes = "STM:sp".getBytes(Charset.defaultCharset());
            BluetoothConnectionService.write(bytes);
            Toast.makeText(Arena.this, "Shortest Path Started.", Toast.LENGTH_LONG).show();
            updateStatusWindow("Auto-Movement Ongoing");

        } else {
            Toast.makeText(Arena.this, "Please connect to Bluetooth.", Toast.LENGTH_LONG).show();
        }
    }

    private void resetObstaclesButton() {
        Chronometer IRTimer = (Chronometer) findViewById(R.id.IRTimer);  // Reset Timer
        IRTimer.setBase(SystemClock.elapsedRealtime());
        IRTimer.stop();
        updateStatusWindow("Ready To Start");



        // Hard coded
        obstacle1.setTranslationX(0);
        obstacle1.setTranslationY(0);

        obstacle2.setTranslationX(0);
        obstacle2.setTranslationY(0);

        obstacle3.setTranslationX(0);
        obstacle3.setTranslationY(0);

        obstacle4.setTranslationX(0);
        obstacle4.setTranslationY(0);

        obstacle5.setTranslationX(0);
        obstacle5.setTranslationY(0);

        obstacle6.setTranslationX(0);
        obstacle6.setTranslationY(0);

        obstacle7.setTranslationX(0);
        obstacle7.setTranslationY(0);

        obstacle8.setTranslationX(0);
        obstacle8.setTranslationY(0);

        car.setX(0);
        car.setY(680);
        car.setRotation(0);
        updateXYDirText();

        obstacle1.setImageResource(resources.get("o1n"));
        obstacle1.setTag(resources.get("o1n"));
        obstacle2.setImageResource(resources.get("o2n"));
        obstacle2.setTag(resources.get("o2n"));
        obstacle3.setImageResource(resources.get("o3n"));
        obstacle3.setTag(resources.get("o3n"));
        obstacle4.setImageResource(resources.get("o4n"));
        obstacle4.setTag(resources.get("o4n"));
        obstacle5.setImageResource(resources.get("o5n"));
        obstacle5.setTag(resources.get("o5n"));
        obstacle6.setImageResource(resources.get("o6n"));
        obstacle6.setTag(resources.get("o6n"));
        obstacle7.setImageResource(resources.get("o7n"));
        obstacle7.setTag(resources.get("o7n"));
        obstacle8.setImageResource(resources.get("o8n"));
        obstacle1.setTag(resources.get("o8n"));

        obstacle1.setRotation(0);
        obstacle2.setRotation(0);
        obstacle3.setRotation(0);
        obstacle4.setRotation(0);
        obstacle5.setRotation(0);
        obstacle6.setRotation(0);
        obstacle7.setRotation(0);
        obstacle8.setRotation(0);

        Toast.makeText(this, "Map Reset", Toast.LENGTH_LONG).show();
    }

    private void setPreset1Button() {
        updateStatusWindow("Ready To Start");

        obstacle1.setX(600);
        obstacle1.setY(200);
        obstacle1.setRotation(180);
        obstacle1.setImageResource(resources.get("o1s"));

        obstacle2.setX(600);
        obstacle2.setY(600);
        obstacle2.setImageResource(resources.get("o2n"));

        obstacle3.setX(480);
        obstacle3.setY(400);
        obstacle3.setRotation(90);
        obstacle3.setImageResource(resources.get("o3e"));

        obstacle4.setX(240);
        obstacle4.setY(280);
        obstacle4.setRotation(270);
        obstacle4.setImageResource(resources.get("o4w"));

        obstacle5.setX(240);
        obstacle5.setY(480);
        obstacle5.setRotation(180);
        obstacle5.setImageResource(resources.get("o5s"));

        Toast.makeText(this, "Preset 1 Applied", Toast.LENGTH_LONG).show();
    }

    private void setPreset2Button() {
        updateStatusWindow("Ready To Start");

        obstacle1.setX(280);
        obstacle1.setY(360);
        obstacle1.setRotation(180);
        obstacle1.setImageResource(resources.get("o1s"));

        obstacle2.setX(720);
        obstacle2.setY(560);
        obstacle2.setRotation(270);
        obstacle2.setImageResource(resources.get("o2w"));

        obstacle3.setX(480);
        obstacle3.setY(400);
        obstacle3.setRotation(90);
        obstacle3.setImageResource(resources.get("o3e"));

        Toast.makeText(this, "Preset 2 Applied", Toast.LENGTH_LONG).show();
    }

    private void setPreset3Button() {
        updateStatusWindow("Ready To Start");

        obstacle1.setX(200);
        obstacle1.setY(360);
        obstacle1.setRotation(270);
        obstacle1.setImageResource(resources.get("o1w"));

        obstacle2.setX(400);
        obstacle2.setY(160);
        obstacle2.setRotation(270);
        obstacle2.setImageResource(resources.get("o2w"));

        obstacle3.setX(720);
        obstacle3.setY(320);
        obstacle3.setRotation(0);
        obstacle3.setImageResource(resources.get("o3n"));

        obstacle4.setX(400);
        obstacle4.setY(520);
        obstacle4.setRotation(180);
        obstacle4.setImageResource(resources.get("o4s"));

        Toast.makeText(this, "Preset 3 Applied", Toast.LENGTH_LONG).show();
    }

    private String getObstacleString(ImageView obstacle) {
        if((int) (obstacle.getX() / 40) > 19 || ((int) obstacle.getY() / 40) > 19){
            return "";
        }
        else {
            return ((int) obstacle.getX() / 40) + "," + ((int) obstacle.getY() / 40) + "," + getImageOrientation(obstacle) + ",";
        }
    }

    private String getImageOrientation(ImageView obstacle) {
        switch (((int) ((obstacle.getRotation() / 90) % 4 + 4) % 4)) {
            case 0:
                return "N";
            case 1:
                return "E";
            case 2:
                return "S";
            case 3:
                return "W";
            default:
                return "X";
        }
    }

    private void updateStatusWindow(String msg) {
        statusWindow.setText(msg);
        Log.d("status Window is ",msg);
    }

    private void updateRobotPosition(int x, int y, int direction) {
        car.setX(x * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL);
        car.setY(y * SNAP_GRID_INTERVAL - SNAP_GRID_INTERVAL);
        switch (direction) {
            case 7: // North-west
                car.setRotation(315);
                break;
            case 0: // North
                car.setRotation(0);
                break;
            case 1: // North-east
                car.setRotation(45);
                break;
            case 2: // East
                car.setRotation(90);
                break;
            case 3: // South-east
                car.setRotation(135);
                break;
            case 4: // South
                car.setRotation(180);
                break;
            case 5: // South-west
                car.setRotation(225);
                break;
            case 6: // West
                car.setRotation(270);
                break;
            default:
                // Shouldn't reach this case
                break;
        }

        updateXYDirText();
    }

    private void updateXYDirText() {
        int x = (int) (car.getX() + SNAP_GRID_INTERVAL)/SNAP_GRID_INTERVAL;
        int y = (int) (car.getY() + SNAP_GRID_INTERVAL)/SNAP_GRID_INTERVAL;
        car_x.setText(String.valueOf(x));
        car_y.setText(String.valueOf(y));

        int direction = (int)car.getRotation();

        if (direction == 315)
            car_dir.setText("North-West");
        else if (direction == 0)
            car_dir.setText("North");
        else if (direction == 45)
            car_dir.setText("North-East");
        else if (direction == 90)
            car_dir.setText("East");
        else if (direction == 135)
            car_dir.setText("South-East");
        else if (direction == 180)
            car_dir.setText("South");
        else if (direction == 225)
            car_dir.setText("South-West");
        else if (direction == 270)
            car_dir.setText("West");
        else
            car_dir.setText("None");
    }

    // Broadcast Receiver for incoming message
    BroadcastReceiver myReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String message = intent.getStringExtra("receivedMessage");
            Log.d("Received message: ", message);
            Log.d("check", "printing of received msg done" );

            String command = message.substring(0, message.indexOf(','));

            switch (command) {
                case "ROBOT":
                    int startingIndex = message.indexOf("<");
                    int endingIndex = message.indexOf(">");
                    String x = message.substring(startingIndex + 1, endingIndex);

                    startingIndex = message.indexOf("<", endingIndex+1);
                    endingIndex = message.indexOf(">", endingIndex+1);
                    String y = message.substring(startingIndex+1, endingIndex);

                    startingIndex = message.indexOf("<", endingIndex+1);
                    endingIndex = message.indexOf(">", endingIndex+1);
                    String direction = message.substring(startingIndex+1, endingIndex);

                    Log.d("ROBOT", "(x: " + x + ") (y: " + y + ") (direction: " + direction + ")");

                    int direction_int=0;
                    switch(direction){
                        case "N":
                            direction_int=0;
                            break;
                        case "NE":
                            direction_int=1;
                            break;
                        case "E":
                            direction_int=2;
                            break;
                        case "SE":
                            direction_int=3;
                            break;
                        case "S":
                            direction_int=4;
                            break;
                        case "SW":
                            direction_int=5;
                            break;
                        case "W":
                            direction_int=6;
                            break;
                        case "NW":
                            direction_int=7;
                            break;
                        default:
                            break;
                    }

                    updateRobotPosition(Integer.parseInt(x), Integer.parseInt(y), direction_int);
                    break;
                case "TARGET":
                    int obstacleNumber = Character.getNumericValue(message.charAt(7));
                    String solution = message.substring(9);
                    setObstacleImage(obstacleNumber,solution);
                    Toast.makeText(Arena.this, "Obstacle "+obstacleNumber+"changed to Target ID: "+ solution, Toast.LENGTH_SHORT).show();

                    break;
                case "STATUS":
                    String msg = " ";
                    if(message.contains("\n")){
                        msg = message.substring(message.indexOf(',')+1, message.indexOf('\n'));
                }else{
                        msg=message.substring(message.indexOf(',')+1);
                    }
                    if(message.contains("STOP")){
                        Chronometer IRTimer = (Chronometer) findViewById(R.id.IRTimer);
                        IRTimer.stop();
                        updateStatusWindow("IR Completed");
                    }
                    updateStatusWindow(msg);
                    break;

                case "MOVE":
                    String moveCommand = message.substring(message.indexOf(',')+1);  // substring after MOVE (w10n)
                    if (moveCommand.length() > 2){ // Forward and Reverse commands
                        // Split moveCommand (eg w10n) into direction (w) + interval (how many 10s) + 'n'
                        String moveDirection = moveCommand.substring(0, 1); // w
                        Log.d("this", moveDirection);
                        String moveInterval = moveCommand.substring(1, moveCommand.length()-1); // doesn't take n, we can leave that out of any consideration
                        int intervals = Integer.parseInt(moveInterval) / 10;  // intervals = how many units of 10s we have to move (10 = 1 grid block)
                        switch(moveDirection){
                            case "w":
                                for (int i = 0; i <intervals; i++) {
                                    forwardButtonCommand();
                                }
                                break;
                            case "s":
                                for (int i = 0; i < intervals; i++){
                                    reverseButtonCommand();
                                }
                                break;
                            default:
                                Log.d("Move command", "Direction is not valid");
                                break;
                        }
                    }

                    else{
                        switch (moveCommand){  // Turn commands
                            case "ln":  // forward left (w, w, a, w, w)
                                forwardButtonCommand();
                                forwardButtonCommand();
                                leftButtonCommand();
                                forwardButtonCommand();
                                forwardButtonCommand();
                                break;
                            case "rn":  //forward right (w, w, d, w, w)
                                forwardButtonCommand();
                                forwardButtonCommand();
                                rightButtonCommand();
                                forwardButtonCommand();
                                forwardButtonCommand();
                                break;
                            case "Ln":  // reverse left (s, s, d, s, s)
                                reverseButtonCommand();
                                reverseButtonCommand();
                                rightButtonCommand();
                                reverseButtonCommand();
                                reverseButtonCommand();
                                break;
                            case "Rn":  //  reverse right (s, s, a, s, s)
                                reverseButtonCommand();
                                reverseButtonCommand();
                                leftButtonCommand();
                                reverseButtonCommand();
                                reverseButtonCommand();
                                break;
                            default:
                                Log.d("Move command", "Command is not a valid turn");
                                Log.d("this", moveCommand);  // checking value of moveCommand
                                break;
                        }
                    }
                default:  // for outer "ROBOT/TARGET/STATUS cases
                    break;
            }

        }
    };
}
