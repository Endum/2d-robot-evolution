/*-
 * ========================LICENSE_START=================================
 * mrsim2d-core
 * %%
 * Copyright (C) 2020 - 2023 Eric Medvet
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
package io.github.ericmedvet.robotevo2d.main.agents.selfassemblyvsr;

import io.github.ericmedvet.mrsim2d.core.ActionPerformer;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.actions.CreateVoxel;
import io.github.ericmedvet.mrsim2d.core.bodies.Body;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.mrsim2d.core.engine.ActionException;
import java.util.List;

public abstract class AbstractSelfAssemblyVSR implements EmbodiedAgent {

  protected final int unitNumber;
  private final Voxel.Material material;
  protected final List<Voxel> unitBody;
  private final double voxelSideLength;
  private final double voxelMass;

  public AbstractSelfAssemblyVSR(
      int unitNumber, Voxel.Material material, double voxelSideLength, double voxelMass) {
    this.unitNumber = unitNumber;
    this.material = material;
    this.unitBody = List.of();
    this.voxelSideLength = voxelSideLength;
    this.voxelMass = voxelMass;
  }

  @Override
  public void assemble(ActionPerformer actionPerformer) throws ActionException {
    for (int i = 0; i < this.unitNumber; i++) {
      Voxel body =
          actionPerformer
              .perform(new CreateVoxel(this.voxelSideLength, this.voxelMass, this.material))
              .outcome()
              .orElseThrow();

      // TODO: Missing initial position of voxels.

      this.unitBody.add(body);
    }
  }

  @Override
  public List<Body> bodyParts() {
    return this.unitBody.stream().map(a -> (Body) a).toList();
  }
}
