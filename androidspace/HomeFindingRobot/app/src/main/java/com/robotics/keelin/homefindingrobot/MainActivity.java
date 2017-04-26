package com.robotics.keelin.homefindingrobot;

import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.os.Handler;
import android.os.ParcelUuid;
import android.support.constraint.ConstraintLayout;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.SurfaceView;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;

import android.widget.AdapterView;
import android.widget.Button;
import android.widget.ListView;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;

import android.widget.TextView;
import android.widget.Toast;
import java.util.ArrayList;

import android.widget.ArrayAdapter;
import android.content.Intent;

@SuppressWarnings("SpellCheckingInspection")
public class MainActivity extends AppCompatActivity implements JoystickView.JoystickListener{
    static final double  JOY_X_MAX     = 910.0;
    static final double  JOY_X_MIN     = 50.0;
    static final double  JOY_Y_MAX     = 810.0;
    static final double  JOY_Y_MIN     = 70.0;
    static final double  JOY_X_NORM    = 525.0;
    static final double  JOY_Y_NORM    = 515.0;

    static final int  PKT_JOYSTICK_X   = 0;
    static final int  PKT_JOYSTICK_Y   = 1;
    static final int  PKT_DISTANCE     = 2;
    static final int  PKT_HEADING      = 3;
    static final int  PKT_DRIFT        = 4;
    static final int  PKT_STRING       = 5;
    static final int  PKT_SET_HOME     = 6;
    static final int  PKT_GO_HOME      = 7;
    static final int  PKT_AUTO_MODE    = 8;
    static final int  PKT_STOP         = 9;
    static final int  PKT_POSITION     = 10;
    static final int  PKT_CONTROL_MODE = 11;
    static final int  PKT_LINE         = 12;
    static final int  PKT_RIGHT        = 13;
    static final int  PKT_LEFT         = 14;

    int joystickValueX = 0, joystickValueY = 0, joystickRcvX = 0, joystickRcvY = 0,distance = 0, positionX = 0, positionY = 0;
    double heading = 0, drift = 0;
    ArrayList<String> sendQueue;

    Button b1,b2,b3,b4;
    private BluetoothAdapter BA;
    ListView lv;
    Handler handler;

    TextView distance_textview, joystick_textview, heading_textview, drift_textview, position_textview;

    private BluetoothSocket socket;
    private BluetoothDevice connectedDevice;
    private InputStream is;
    private OutputStream os;
    private boolean CONTINUE_READ_WRITE = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //noinspection unused
        JoystickView joystick = new JoystickView(this);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        sendQueue = new ArrayList<>();

        b1 = (Button)findViewById(R.id.button);
        b2 = (Button)findViewById(R.id.button2);
        b3 = (Button)findViewById(R.id.button3);
        b4 = (Button)findViewById(R.id.button4);

        handler = new Handler();

        ConstraintLayout clayout = (ConstraintLayout) findViewById(R.id.include_robot);
        distance_textview = (TextView) clayout.findViewById(R.id.distance_textview);
        joystick_textview = (TextView) clayout.findViewById(R.id.joystick_textview);
        heading_textview = (TextView) clayout.findViewById(R.id.heading_textview);
        drift_textview = (TextView) clayout.findViewById(R.id.drift_textview);
        position_textview = (TextView) clayout.findViewById(R.id.position_textview);

        BA = BluetoothAdapter.getDefaultAdapter();
        lv = (ListView)findViewById(R.id.listView);

        lv.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view,
                                    int position, long id) {
                Toast.makeText(getApplicationContext(),
                        "Click ListItem Number " + position, Toast.LENGTH_LONG)
                        .show();

                Set<BluetoothDevice>pairedDevices= BA.getBondedDevices();
                for(BluetoothDevice bt : pairedDevices)
                    if (bt.getName().equals(parent.getItemAtPosition(position))) {
                        connectedDevice = bt;
                        CONTINUE_READ_WRITE = true;
                        new Thread(reader).start();
                        ConstraintLayout layout1 = (ConstraintLayout) findViewById(R.id.include_bt);
                        layout1.setVisibility(View.INVISIBLE);
                        ConstraintLayout layout2 = (ConstraintLayout) findViewById(R.id.include_robot);
                        layout2.setVisibility(View.VISIBLE);
                        SurfaceView layout3 = (SurfaceView) findViewById(R.id.joystick);
                        layout3.setVisibility(View.VISIBLE);
                        break;
                    }
            }
        });
    }

    public void on(View v){
        if (!BA.isEnabled()) {
            Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnOn, 0);
            Toast.makeText(getApplicationContext(), "Turned on",Toast.LENGTH_LONG).show();
        } else {
            Toast.makeText(getApplicationContext(), "Already on", Toast.LENGTH_LONG).show();
        }
    }

    public void off(View v){
        BA.disable();
        if(socket != null){
            try{
                is.close();
                os.close();
                socket.close();
            }catch(Exception e){e.printStackTrace();}
            CONTINUE_READ_WRITE = false;
        }
        Toast.makeText(getApplicationContext(), "Turned off" ,Toast.LENGTH_LONG).show();
    }

    public void visible(View v){
        Intent getVisible = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        startActivityForResult(getVisible, 0);
    }

    public void list(View v){
        Set<BluetoothDevice>pairedDevices= BA.getBondedDevices();

        ArrayList<String> list = new ArrayList<>();

        for(BluetoothDevice bt : pairedDevices) list.add(bt.getName());
        Toast.makeText(getApplicationContext(), "Showing Paired Devices",Toast.LENGTH_SHORT).show();

        final ArrayAdapter<String> adapter = new  ArrayAdapter<>(this,android.R.layout.simple_list_item_1, list);

        lv.setAdapter(adapter);
    }

    public void set_home(View v) {
        sendMessage(PKT_SET_HOME, "");
    }

    public void go_home(View v) {
        sendMessage(PKT_GO_HOME, "");
    }

    public void auto_run(View v) {
        sendMessage(PKT_AUTO_MODE, "");
    }

    public void control_mode(View v) {
        sendMessage(PKT_CONTROL_MODE, "");
    }

    public void stop(View v) {
        sendMessage(PKT_STOP, "");
    }

    public void left(View v) {
        sendMessage(PKT_LEFT, "");
    }

    public void right(View v) {
        sendMessage(PKT_RIGHT, "");
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            CONTINUE_READ_WRITE = false;
            connectedDevice = null;
            ConstraintLayout layout1 = (ConstraintLayout) findViewById(R.id.include_bt);
            layout1.setVisibility(View.VISIBLE);
            ConstraintLayout layout2 = (ConstraintLayout) findViewById(R.id.include_robot);
            layout2.setVisibility(View.INVISIBLE);
            SurfaceView layout3 = (SurfaceView) findViewById(R.id.joystick);
            layout3.setVisibility(View.INVISIBLE);
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    protected void sendMessage(int type, String message) {
        sendQueue.add(type+":"+message);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if(socket != null){
            try{
                is.close();
                os.close();
                socket.close();
            }catch(Exception e){e.printStackTrace();}
            CONTINUE_READ_WRITE = false;
        }
    }

    private Runnable reader = new Runnable() {
        public void run() {
            ParcelUuid[] uuids = connectedDevice.getUuids();
            try {
                socket = connectedDevice.createRfcommSocketToServiceRecord(uuids[0].getUuid());
                socket.connect();
                is = socket.getInputStream();
                os = socket.getOutputStream();
                new Thread(writer).start();

                int bytesRead, val = 0, stage = 0;
                int length = 0, type = -1, cur = 0;
                String result = "";
                //Keep reading the messages while connection is open...
                while(!Thread.currentThread().isInterrupted() && CONTINUE_READ_WRITE){
                    bytesRead = is.available();
                    if (bytesRead > 0) {
                        final StringBuilder sb = new StringBuilder();
                        while(true) {
                            bytesRead = is.read();
                            if (stage == 0) {
                                length = (bytesRead&0xf);
                                type = (bytesRead>>4);
                                if (length != 0) {
                                    stage = 1;
                                    cur = 0;
                                    result = "";
                                } else break;
                            } else {
                                if (type == PKT_DISTANCE) {
                                    if (cur == 0)
                                        val = bytesRead<<8;
                                    else {
                                        val |= bytesRead;
                                        distance = val;
                                        handler.post(new Runnable() {
                                            public void run() {
                                                distance_textview.setText("" + getString(R.string.distance_str) + distance);
                                            }
                                        });
                                    }
                                } else if (type == PKT_JOYSTICK_X) {
                                    if (cur == 0)
                                        val = bytesRead;
                                    else {
                                        val |= bytesRead<<8;
                                        joystickRcvX = val;
                                        handler.post(new Runnable() {
                                            public void run() {
                                                joystick_textview.setText("" + getString(R.string.joy_val_str) + joystickRcvX + ", " + joystickRcvY + ")");
                                            }
                                        });
                                        sendMessage(PKT_JOYSTICK_X, "" + (char)(joystickValueX&0xff) + "" + (char)((joystickValueX>>8)&0xff));
                                    }
                                } else if (type == PKT_JOYSTICK_Y) {
                                    if (cur == 0)
                                        val = bytesRead;
                                    else {
                                        val |= bytesRead<<8;
                                        joystickRcvY = val;
                                        handler.post(new Runnable() {
                                            public void run() {
                                                joystick_textview.setText("" + getString(R.string.joy_val_str) + joystickRcvX + ", " + joystickRcvY + ")");
                                            }
                                        });
                                        sendMessage(PKT_JOYSTICK_Y, "" + (char)(joystickValueY&0xff) + "" + (char)((joystickValueY>>8)&0xff));
                                    }
                                } else if (type == PKT_HEADING) {
                                    result += (char) bytesRead;
                                    if (cur == length-1) {
                                        heading = Double.parseDouble(result.replace(' ', (char)0));
                                        handler.post(new Runnable() {
                                            public void run() {
                                                heading_textview.setText("" + getString(R.string.heading_str) + heading);
                                            }
                                        });
                                    }
                                } else if (type == PKT_DRIFT) {
                                    result += (char) bytesRead;
                                    if (cur == length-1) {
                                        drift = Double.parseDouble(result.replace(' ', (char)0));
                                        handler.post(new Runnable() {
                                            public void run() {
                                                drift_textview.setText("" + getString(R.string.drift_str) + drift);
                                            }
                                        });
                                    }
                                } else if (type == PKT_POSITION) {
                                    if (cur == 0)
                                        positionX = bytesRead;//((bytesRead&0x80)!=0 ? -1:1)*(int)(bytesRead&0x7f);
                                    else {
                                        positionY = bytesRead;//((bytesRead&0x80)!=0 ? -1:1)*(int)(bytesRead&0x7f);
                                        handler.post(new Runnable() {
                                            public void run() {
                                                position_textview.setText("" + getString(R.string.pos_val_str) + positionX + ", " + positionY + ")");
                                            }
                                        });
                                    }
                                } else result += (char) bytesRead; // PKT_STRING and others

                                cur++;
                                if (cur == length) {
                                    stage = 0;
                                    break;
                                }
                            }
                        }
                        if (type == PKT_STRING) {
                            if (result.equals("begin")){
                                sendMessage(PKT_JOYSTICK_X, "" + (char) ((int)JOY_X_NORM & 0xff) + "" + (char) (((int)JOY_X_NORM >> 8) & 0xff));
                                sendMessage(PKT_JOYSTICK_Y, "" + (char) ((int)JOY_Y_NORM & 0xff) + "" + (char) (((int)JOY_Y_NORM >> 8) & 0xff));
                            }
                            sb.append(result);

                            //Show message on UIThread
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    Toast.makeText(getApplicationContext(), sb.toString(), Toast.LENGTH_LONG).show();
                                }
                            });
                        }
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
                CONTINUE_READ_WRITE = false;
            }
        }
    };

    private Runnable writer = new Runnable() {

        @Override
        public void run() {
            while(CONTINUE_READ_WRITE){
                if(!sendQueue.isEmpty()) {
                    try {
                        String msg = sendQueue.remove(0);
                        int type;
                        // Assuming two digit type max
                        if (msg.charAt(1) == ':')
                            type = Integer.parseInt(msg.charAt(0) + "");
                        else
                            type = Integer.parseInt(msg.charAt(0) + "" + msg.charAt(1) + "");
                        msg = msg.substring(msg.indexOf(':') + 1);
                        os.write((type<<4)|(msg.length()&0xf));
                        for (char c : msg.toCharArray())
                            os.write(c);
                        os.flush();
                        Thread.sleep(200);
                    } catch (Exception e) {
                        e.printStackTrace();
                        CONTINUE_READ_WRITE = false;
                    }
                }
            }
        }
    };

    @Override
    public void onJoystickMoved(float xPercent, float yPercent, int id) {
        switch (id) {
            case R.id.joystick:
                if (xPercent > 0)
                    joystickValueX = (int) (xPercent * (JOY_X_MAX - JOY_X_NORM) + JOY_X_NORM);
                else
                    joystickValueX = (int) (xPercent * (JOY_X_NORM - JOY_X_MIN) + JOY_X_NORM);
                if (yPercent > 0)
                    joystickValueY = (int) (yPercent * (JOY_Y_MAX - JOY_Y_NORM) + JOY_Y_NORM);
                else
                    joystickValueY = (int) (yPercent * (JOY_Y_NORM - JOY_Y_MIN) + JOY_Y_NORM);
                break;
        }
    }
}
