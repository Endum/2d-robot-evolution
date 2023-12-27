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

import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.jsdynsym.buildable.builders.NumericalDynamicalSystems;
import io.github.ericmedvet.jsdynsym.core.numerical.MultivariateRealFunction;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.mrsim2d.core.Sensor;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.robotevo2d.main.agents.selfassemblyvsr.NumSelfAssemblyVSR;
import java.util.List;
import java.util.stream.Stream;

@Discoverable(prefixTemplate = "sim|s.agent|a")
public class Agents {

  private Agents() {}

  @SuppressWarnings({"unused", "unchecked"})
  public static NumSelfAssemblyVSR numSelfAssemblyVSR(
      @Param("unitNumber") int unitNumber,
      @Param(value = "voxelSideLength", dD = 1.0) double voxelSideLength,
      @Param(value = "voxelMass", dD = 1.0) double voxelMass,
      @Param(value = "nSignals", dI = 1) int nSignals,
      @Param("function") NumericalDynamicalSystems.Builder<?, ?> numericalDynamicalSystemBuilder,
      @Param("sensors") List<Sensor<? super Voxel>> sensors) {
    return new NumSelfAssemblyVSR(
        unitNumber,
        new Voxel.Material(),
        voxelSideLength,
        voxelMass,
        nSignals,
        (List<NumericalDynamicalSystem<?>>) Stream.generate(() -> numericalDynamicalSystemBuilder.apply(
                MultivariateRealFunction.varNames(
                    "x", NumSelfAssemblyVSR.nOfInputs(sensors.size(), nSignals)),
                MultivariateRealFunction.varNames("y", NumSelfAssemblyVSR.nOfOutputs(nSignals))))
            .limit(unitNumber)
            .toList(),
        Stream.generate(() -> List.copyOf(sensors)).limit(unitNumber).toList());
  }
}
