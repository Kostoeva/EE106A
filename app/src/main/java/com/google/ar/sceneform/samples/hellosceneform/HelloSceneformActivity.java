/*
 * Copyright 2018 Google LLC. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.ar.sceneform.samples.hellosceneform;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.os.Build;
import android.os.Build.VERSION_CODES;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import com.google.ar.core.Anchor;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
import com.google.ar.core.Pose;
import com.google.ar.sceneform.AnchorNode;
import com.google.ar.sceneform.rendering.ModelRenderable;
import com.google.ar.sceneform.ux.ArFragment;
import com.google.ar.sceneform.ux.TransformableNode;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

/**
 * This is an example activity that uses the Sceneform UX package to make common AR tasks easier.
 */
public class HelloSceneformActivity extends AppCompatActivity {
  private static final String TAG = HelloSceneformActivity.class.getSimpleName();
  private static final double MIN_OPENGL_VERSION = 3.0;

  private ArFragment arFragment;
  private ModelRenderable andyRenderable;

  private TextView debugText;

  private float x;
  private float y;
  private float z;

  // List of waypoints: waypoint = [x, z]
  private ArrayList<float[]> waypoints = new ArrayList<>();

  // Clear button
  private Button clearButton;

  // Finish button
  private Button finishButton;

  @Override
  @SuppressWarnings({"AndroidApiChecker", "FutureReturnValueIgnored"})
  // CompletableFuture requires api level 24
  // FutureReturnValueIgnored is not valid
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    if (!checkIsSupportedDeviceOrFinish(this)) {
      return;
    }

    setContentView(R.layout.activity_ux);

    // Set clear button.
    clearButton = findViewById(R.id.clearButton);
      clearButton.setOnClickListener(new View.OnClickListener() {
          @Override
          public void onClick(View view)
          {
              waypoints = new ArrayList<>();
              debugText.setText("Cleared waypoints");
              System.out.println("clearing waypoints -----------------------------------");
          }
      });

      // Set finish button.
    finishButton = findViewById(R.id.finishButton);
      finishButton.setOnClickListener(new View.OnClickListener() {
          @Override
          public void onClick(View view)
          {
              debugText.setText("Finished: waypoints sent to client");
              writeFileOnInternalStorage(getApplicationContext(), "ee106a-waypoints", waypoints);
              System.out.println("finished -----------------------------------");
          }
      });

    // DebugText: hit pose.
    debugText = findViewById(R.id.debugText);

    arFragment = (ArFragment) getSupportFragmentManager().findFragmentById(R.id.ux_fragment);

    // When you build a Renderable, Sceneform loads its resources in the background while returning
    // a CompletableFuture. Call thenAccept(), handle(), or check isDone() before calling get().
    ModelRenderable.builder()
        .setSource(this, R.raw.andy)
        .build()
        .thenAccept(renderable -> andyRenderable = renderable)
        .exceptionally(
            throwable -> {
              Toast toast =
                  Toast.makeText(this, "Unable to load andy renderable", Toast.LENGTH_LONG);
              toast.setGravity(Gravity.CENTER, 0, 0);
              toast.show();
              return null;
            });

    arFragment.setOnTapArPlaneListener(
        (HitResult hitResult, Plane plane, MotionEvent motionEvent) -> {
          if (andyRenderable == null) {
            return;
          }

          // Create the Anchor.
          Anchor anchor = hitResult.createAnchor();

          System.out.println("-------------------------");
          System.out.println(hitResult.getHitPose().toString());

          Pose pose = hitResult.getHitPose();
          x = pose.tx();
          y = pose.ty();
          z = pose.tz();

          // Add current waypoint to list of waypoints.
          float[] current_waypoint = new float[2];
          current_waypoint[0] = x;
          current_waypoint[1] = z;
          waypoints.add(current_waypoint);

          debugText.setText(hitResult.getHitPose().toString());

          AnchorNode anchorNode = new AnchorNode(anchor);
          anchorNode.setParent(arFragment.getArSceneView().getScene());

          // Create the transformable andy and add it to the anchor.
          TransformableNode andy = new TransformableNode(arFragment.getTransformationSystem());
          andy.setParent(anchorNode);
          andy.setRenderable(andyRenderable);
          andy.select();
        });
  }

  /**
   * Returns false and displays an error message if Sceneform can not run, true if Sceneform can run
   * on this device.
   *
   * <p>Sceneform requires Android N on the device as well as OpenGL 3.0 capabilities.
   *
   * <p>Finishes the activity if Sceneform can not run
   */
  public static boolean checkIsSupportedDeviceOrFinish(final Activity activity) {
    if (Build.VERSION.SDK_INT < VERSION_CODES.N) {
      Log.e(TAG, "Sceneform requires Android N or later");
      Toast.makeText(activity, "Sceneform requires Android N or later", Toast.LENGTH_LONG).show();
      activity.finish();
      return false;
    }
    String openGlVersionString =
        ((ActivityManager) activity.getSystemService(Context.ACTIVITY_SERVICE))
            .getDeviceConfigurationInfo()
            .getGlEsVersion();
    if (Double.parseDouble(openGlVersionString) < MIN_OPENGL_VERSION) {
      Log.e(TAG, "Sceneform requires OpenGL ES 3.0 later");
      Toast.makeText(activity, "Sceneform requires OpenGL ES 3.0 or later", Toast.LENGTH_LONG)
          .show();
      activity.finish();
      return false;
    }
    return true;
  }

  // Write waypoint arraylist to internal storage.
  public void writeFileOnInternalStorage(Context mcoContext, String sFileName, ArrayList<float[]> sBody){
      File file = new File(mcoContext.getFilesDir(),"ee106a");
      if(!file.exists()){
           file.mkdir();
      }

      try{
          File gpxfile = new File(file, sFileName);
          FileWriter writer = new FileWriter(gpxfile);
          for (int i = 0; i < waypoints.size(); i++) {
              writer.append(Float.toString(waypoints.get(i)[0]) + ',' + Float.toString(waypoints.get(i)[1]));
              writer.append('\n');
          }
          writer.flush();
          writer.close();
      }catch (Exception e){
          e.printStackTrace();
      }
  }

}
