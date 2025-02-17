/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2022 - 2023 Eric Medvet
 * %%
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
 * =========================LICENSE_END==================================
 */

package io.github.ericmedvet.robotevo2d.main.builders;

import io.github.ericmedvet.jgea.core.listener.Accumulator;
import io.github.ericmedvet.jgea.core.listener.AccumulatorFactory;
import io.github.ericmedvet.jgea.core.solver.Individual;
import io.github.ericmedvet.jgea.core.solver.POCPopulationState;
import io.github.ericmedvet.jgea.experimenter.Run;
import io.github.ericmedvet.jgea.experimenter.RunOutcome;
import io.github.ericmedvet.jgea.experimenter.Utils;
import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.jnb.core.ParamMap;
import io.github.ericmedvet.jsdynsym.core.DoubleRange;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.mrsim2d.viewer.Drawer;
import io.github.ericmedvet.mrsim2d.viewer.VideoBuilder;
import io.github.ericmedvet.mrsim2d.viewer.VideoUtils;
import java.io.*;
import java.util.Base64;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.Logger;
import java.util.random.RandomGenerator;
import java.util.stream.Collectors;

@Discoverable(prefixTemplate = "evorobots|er")
public class Misc {

  public static final int FILE_VIDEO_W = 600;
  public static final int FILE_VIDEO_H = 400;
  private static final Logger L = Logger.getLogger(Misc.class.getName());

  private Misc() {}

  @SuppressWarnings("unused")
  public static Function<List<Double>, List<Double>> doublesRandomizer(
      @Param(value = "randomGenerator", dNPM = "sim.defaultRG()") RandomGenerator randomGenerator,
      @Param(value = "range", dNPM = "sim.range(max=1.0;min=-1.0)") DoubleRange range) {
    return values -> values.stream()
        .map(v -> range.denormalize(randomGenerator.nextDouble()))
        .toList();
  }

  @SuppressWarnings("unused")
  public static Function<Object, Object> fromBase64(@Param("s") String s) {
    return o -> {
      try (ByteArrayInputStream bais =
              new ByteArrayInputStream(Base64.getDecoder().decode(s));
          ObjectInputStream ois = new ObjectInputStream(bais)) {
        return ois.readObject();
      } catch (Throwable t) {
        L.warning("Cannot deserialize: %s".formatted(t));
        return null;
      }
    };
  }

  @SuppressWarnings("unused")
  public static Function<Object, Object> fromRunOutcome(
      @Param("filePath") String filePath,
      @Param(value = "index", dI = 0) int index,
      @Param(value = "", injection = Param.Injection.BUILDER) NamedBuilder<?> builder) {
    try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
      RunOutcome runOutcome = (RunOutcome) builder.build(br.lines().collect(Collectors.joining()));
      return fromBase64(runOutcome.serializedGenotypes().get(index));
    } catch (FileNotFoundException e) {
      throw new RuntimeException("Cannot find run outcome file %s".formatted(filePath));
    } catch (IOException e) {
      throw new RuntimeException("Cannot read run outcome file %s due to: %s".formatted(filePath, e));
    }
  }

  @SuppressWarnings("unused")
  public static <A> AccumulatorFactory<POCPopulationState<?, ?, A, ?>, File, Run<?, ?, A, ?>> video(
      @Param(value = "filePathTemplate", dS = "video-{index:%04d}.mp4") String filePathTemplate,
      @Param(value = "titleTemplate", dS = "run.index={index:%04d}") String titleTemplate,
      @Param(value = "w", dI = FILE_VIDEO_W) int w,
      @Param(value = "h", dI = FILE_VIDEO_H) int h,
      @Param(value = "frameRate", dD = 30) double frameRate,
      @Param(value = "startTime", dD = 0) double startTime,
      @Param(value = "endTime", dD = 30) double endTime,
      @Param(value = "codec", dS = "jcodec") VideoUtils.EncoderFacility codec,
      @Param(value = "drawer", dNPM = "sim.drawer()") Function<String, Drawer> drawer,
      @Param("task") Task<A, ?> task,
      @Param(value = "engine", dNPM = "sim.engine()") Supplier<Engine> engineSupplier,
      @Param(value = "individual", dNPM = "ea.nf.best()")
          Function<POCPopulationState<?, ?, A, ?>, Individual<?, A, ?>> individualFunction,
      @Param(value = "", injection = Param.Injection.MAP) ParamMap map) {
    return run -> Accumulator.<POCPopulationState<?, ?, A, ?>>last().then(state -> {
      // extract individual
      A a = individualFunction.apply(state).solution();
      // create file
      boolean tempFile = false;
      File file;
      try {
        if (filePathTemplate.isEmpty()) {
          tempFile = true;
          file = File.createTempFile("video", ".mp4");
          file.deleteOnExit();
        } else {
          String fileName = Utils.interpolate(filePathTemplate, run);
          file = io.github.ericmedvet.jgea.core.util.Misc.checkExistenceAndChangeName(new File(fileName));
        }
        // do video
        String videoName = Utils.interpolate(titleTemplate, run);
        VideoBuilder videoBuilder =
            new VideoBuilder(w, h, startTime, endTime, frameRate, codec, file, drawer.apply(videoName));
        L.info("Doing video for run %d on file %s"
            .formatted(run.index(), tempFile ? "temp" : file.getAbsolutePath()));
        task.run(a, engineSupplier.get(), videoBuilder);
        file = videoBuilder.get();
        L.info("Video done for run %d on file %s"
            .formatted(run.index(), tempFile ? "temp" : file.getAbsolutePath()));
        return file;
      } catch (IOException ex) {
        L.warning("Cannot make video: %s".formatted(ex));
      }
      return null;
    });
  }
}
