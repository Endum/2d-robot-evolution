/*-
 * ========================LICENSE_START=================================
 * robotevo2d-main
 * %%
 * Copyright (C) 2018 - 2024 Eric Medvet
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

import static io.github.ericmedvet.mrsim2d.buildable.builders.Terrains.*;

import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.mrsim2d.core.geometry.Path;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;
import io.github.ericmedvet.mrsim2d.core.geometry.Terrain;

@Discoverable(prefixTemplate = "sim|s.terrain|t")
public class Terrains {
  public static final double START_W = 30d;
  public static final double HOLE_H = 10d;
  public static final double HOLE_W = 5d;
  public static final double END_W = 30d;

  private Terrains() {}

  @SuppressWarnings("unused")
  public static Terrain holehill(
      @Param(value = "startW", dD = START_W) double startW,
      @Param(value = "holeH", dD = HOLE_H) double holeH,
      @Param(value = "holeW", dD = HOLE_W) double holeW,
      @Param(value = "endW", dD = END_W) double endW,
      @Param(value = "borderW", dD = BORDER_W) double borderW,
      @Param(value = "borderH", dD = BORDER_H) double borderH) {

    Path p = new Path(new Point(startW, 0))
        .moveBy(0, -holeH)
        .moveBy(holeW, 0)
        .moveBy(0, holeH)
        .moveBy(endW, 0);

    return Terrain.fromPath(p, H, borderW, borderH);
  }
}
