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

import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.jsdynsym.core.DoubleRange;
import io.github.ericmedvet.mrsim2d.core.tasks.Outcome;
import java.util.function.Function;

@Discoverable(prefixTemplate = "sim|s.task.piling|p")
public class CustomOutcomeFunctions {
  private CustomOutcomeFunctions() {}

  @SuppressWarnings("unused")
  public static Function<Outcome<?>, Double> avgUnitH(@Param(value = "transientTime", dD = 0) double transientTime) {
    return o -> o.subOutcome(new DoubleRange(transientTime, o.duration())).getObservations().values().stream()
        .mapToDouble(ob -> ob.getAgents().stream()
            .mapToDouble(a -> a.polies().stream()
                .mapToDouble(u -> u.center().y())
                .average()
                .orElse(0))
            .average()
            .orElse(0))
        .average()
        .orElse(0);
  }

  @SuppressWarnings("unused")
  public static Function<Outcome<?>, Double> mdnUnitH(@Param(value = "transientTime", dD = 0) double transientTime) {
    return o -> o.subOutcome(new DoubleRange(transientTime, o.duration())).getObservations().values().stream()
        .mapToDouble(ob -> ob.getAgents().stream()
            .mapToDouble(a -> a.polies().stream()
                .mapToDouble(u -> u.center().y())
                .sorted()
                .toArray()[ob.getAgents().size() / 2])
            .average()
            .orElse(0))
        .average()
        .orElse(0);
  }
}
