package com._604robotics.robotnik.prefabs.auto;

import com._604robotics.robotnik.DashboardManager;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class QuikPlanSwerveReader {
  List<Double> timeData = new ArrayList<>();
  List<List<Double>> data = new ArrayList<>();
  List<Boolean> shootData = new ArrayList<>();
  Output<File> chooser;

  public QuikPlanSwerveReader(Module module) {
    clearLoadedData();
    List<File> files = new ArrayList<>();
    try {
      Files.walk(Filesystem.getDeployDirectory().toPath())
          .map(Path::toFile)
          .filter((file) -> file.getName().endsWith(".csv"))
          .forEach(files::add);
    } catch (IOException e) {
      e.printStackTrace();
    }

    System.out.println(files);
    this.chooser =
        DashboardManager.getInstance()
            .registerCollection("Quikplan Paths", files.get(0), files, module);
    loadChosenFile();
  }

  public void loadData(File file) {
    clearLoadedData();
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.isEmpty()) {
          double[] values =
              Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
          List<Double> classValues = new ArrayList<>();
          for (Double d : values) {
            classValues.add(d);
          }
          this.timeData.add(classValues.get(0));
          for (TrajectoryState state : TrajectoryState.values()) {
            this.data.get(state.index).add(classValues.get(state.index + 1));
          }
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void loadChosenFile() {
    loadData(chooser.get());
    System.out.println("Loading: " + chooser.get());
  }

  public void clearLoadedData() {
    timeData.clear();
    data.clear();

    for (int i = 0; i <= TrajectoryState.values().length - 1; i++) {
      data.add(new ArrayList<Double>());
    }
  }

  public List<Double> getState(double time) {
    List<Double> state = new ArrayList<>();
    for (List<Double> dataList : data) {
      LinearInterpolator li = new LinearInterpolator();
      PolynomialSplineFunction psf =
          li.interpolate(
              timeData.stream().mapToDouble(d -> d).toArray(),
              dataList.stream().mapToDouble(d -> d).toArray());
      state.add(psf.value(Math.min(time, getTotalTime())));
    }
    return state;
  }

  public boolean doShoot(double time) {
    // System.out.println(getState(time).get(TrajectoryState.shoot.index));
    return getState(time).get(TrajectoryState.shoot.index) == 1.0;
  }

  public double getTotalTime() {
    return timeData.get(timeData.size() - 1);
  }

  public enum TrajectoryState {
    x(0),
    y(1),
    Theta(2),
    dx(3),
    dy(4),
    dTheta(5),
    shoot(6);

    public final int index;

    private TrajectoryState(int index) {
      this.index = index;
    }
  }
}
