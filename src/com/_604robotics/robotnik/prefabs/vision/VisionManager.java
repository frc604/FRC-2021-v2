package com._604robotics.robotnik.prefabs.vision;

import com._604robotics.robotnik.Reliability;
import java.util.HashMap;
import java.util.Map;

public class VisionManager {
  private static VisionManager single_instance = null;

  private HashMap<String, VisionCamera> cameras = new HashMap<>();

  public static VisionManager getInstance() {
    if (single_instance == null) single_instance = new VisionManager();

    return single_instance;
  }

  public VisionManager() {}

  public void registerCamera(VisionCamera camera, String name) {
    cameras.put(name, camera);
  }

  public void update() {
    for (Map.Entry<String, VisionCamera> camera : cameras.entrySet()) {
      Reliability.swallowThrowables(
          () -> camera.getValue().pull(), "Error fetching measurement from: " + camera.getKey());
    }
  }
}
