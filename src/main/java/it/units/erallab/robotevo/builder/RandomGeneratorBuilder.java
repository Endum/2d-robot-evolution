/*
 * Copyright 2022 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.units.erallab.robotevo.builder;

import it.units.erallab.mrsim.util.builder.NamedBuilder;
import it.units.erallab.mrsim.util.builder.ParamMap;

import java.util.Random;
import java.util.random.RandomGenerator;

/**
 * @author "Eric Medvet" on 2022/08/11 for 2d-robot-evolution
 */
public class RandomGeneratorBuilder extends NamedBuilder<RandomGenerator> {

  private RandomGeneratorBuilder() {
    register("default", RandomGeneratorBuilder::createDefault);
  }

  private static RandomGenerator createDefault(ParamMap m, NamedBuilder<?> nb) {
    return new Random(m.i("seed", 1));
  }

  private final static RandomGeneratorBuilder INSTANCE = new RandomGeneratorBuilder();

  public static RandomGeneratorBuilder getInstance() {
    return INSTANCE;
  }

}

