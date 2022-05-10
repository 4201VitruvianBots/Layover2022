package frc.robot.utils;

import frc.robot.Constants.SwerveDrive.ModulePosition;
import java.util.*;

/**
 * A convenience class that maps {@link ModulePosition}s to any class, e.g. module translations,
 * module states, etc.
 *
 * <p>Also contains functions to convert to and from arrays so that it's easier to use WPILib swerve
 * functions.
 */
public class SwerveModuleMap<V> extends HashMap<ModulePosition, V> {
  public SwerveModuleMap() {}

  /**
   * Creates a {@code SwerveModuleMap} with the contents of a {@link Map}.
   *
   * @param map Must have {@link ModulePosition} as the key type
   */
  public SwerveModuleMap(Map<ModulePosition, V> map) {
    for (ModulePosition i : map.keySet()) {
      this.put(i, map.get(i));
    }
  }

  /**
   * Creates a {@code SwerveModuleMap} from multiple values, in the order specified in the {@link
   * ModulePosition} enum.
   *
   * <p>For instantiation, it's better to use {@link #of(ModulePosition, V, ModulePosition, V,
   * ModulePosition, V, ModulePosition, V) of}{@code (K,V,K,V,K,V,K,V)} for clarity. However, it is
   * useful for processing the output of a WPILib swerve function which returns an array.
   *
   * @param values Must have at least as many elements as {@link ModulePosition} has entries. Any
   *     entries after will be ignored.
   */
  @SafeVarargs
  public static <V> SwerveModuleMap<V> of(V... values) {
    SwerveModuleMap<V> map = new SwerveModuleMap<>();
    for (int i = 0; i < ModulePosition.values().length; i++) {
      map.put(ModulePosition.values()[i], values[i]);
    }
    return map;
  }

  /** Creates a {@code SwerveModuleMap} mapping four {@link ModulePosition}s and four values. */
  public static <V> SwerveModuleMap<V> of(
      ModulePosition k1,
      V v1,
      ModulePosition k2,
      V v2,
      ModulePosition k3,
      V v3,
      ModulePosition k4,
      V v4) {
    return new SwerveModuleMap<V>(Map.of(k1, v1, k2, v2, k3, v3, k4, v4));
  }

  /**
   * Warning: This method does not preserve the module order because it returns a {@link
   * java.util.Collection}. Use {@link #values() values} instead.
   */
  @Deprecated
  @Override
  public Collection<V> values() {
    return super.values();
  }

  /**
   * Returns the values from the map as a {@link List} in the same order as in the {@link
   * ModulePosition} enum.
   *
   * <p>Use instead of {@link #values() values} because that returns a {@link java.util.Collection},
   * which is not ordered.
   */
  public List<V> orderedValues() {
    ArrayList<V> list = new ArrayList<>();
    for (ModulePosition i : ModulePosition.values()) {
      list.add(get(i));
    }
    return list;
  }

  /**
   * Returns the values from the map as an {@code Array} in the same order as in the {@link
   * ModulePosition} enum.
   *
   * <p>Useful when a WPILib swerve function requires an array as input.
   *
   * @param array An array of the class to output an array of, e.g. {@code
   *     moduleTranslations.valuesArray(new Translation2d[0])}. Required because Java can't make an
   *     array of generics.
   */
  public V[] valuesArray(V[] array) {
    return orderedValues().toArray(array);
  }
}
