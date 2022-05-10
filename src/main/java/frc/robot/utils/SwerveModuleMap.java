package frc.robot.utils;

import frc.robot.Constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A convenience class that maps {@link Constants.SwerveDrive.ModulePosition}s to any class, e.g. module translations, module states, etc.<p>
 * Also contains functions to convert to and from arrays so that it's easier to use WPILib swerve functions.
 */
public class SwerveModuleMap<V> extends HashMap<Constants.SwerveDrive.ModulePosition, V> {
    public SwerveModuleMap() {
    }

    /**
     * Creates a {@code SwerveModuleMap} with the contents of a {@link Map}.
     *
     * @param map Must have {@link Constants.SwerveDrive.ModulePosition} as the key type
     */
    public SwerveModuleMap(Map<Constants.SwerveDrive.ModulePosition, V> map) {
        for (Constants.SwerveDrive.ModulePosition i : map.keySet()) {
            this.put(i, map.get(i));
        }
    }

    /**
     * Creates a {@code SwerveModuleMap} from multiple values, in the order specified in the {@link Constants.SwerveDrive.ModulePosition} enum.<p>
     * For instantiation, it's better to use {@link #of(Constants.SwerveDrive.ModulePosition, V, Constants.SwerveDrive.ModulePosition, V, Constants.SwerveDrive.ModulePosition, V, Constants.SwerveDrive.ModulePosition, V) of}{@code (K,V,K,V,K,V,K,V)}
     * for clarity. However, it is useful for processing the output of a WPILib swerve function which returns an array.
     *
     * @param values Must have at least as many elements as {@link Constants.SwerveDrive.ModulePosition} has entries. Any entries after will be ignored.
     */
    @SafeVarargs
    public static <V> SwerveModuleMap<V> of(V... values) {
        SwerveModuleMap<V> map = new SwerveModuleMap<>();
        for (int i = 0; i < Constants.SwerveDrive.ModulePosition.values().length; i++) {
            map.put(Constants.SwerveDrive.ModulePosition.values()[i], values[i]);
        }
        return map;
    }

    /**
     * Creates a {@code SwerveModuleMap} mapping four {@link Constants.SwerveDrive.ModulePosition}s and four values.
     */
    public static <V> SwerveModuleMap<V> of(Constants.SwerveDrive.ModulePosition k1, V v1, Constants.SwerveDrive.ModulePosition k2, V v2, Constants.SwerveDrive.ModulePosition k3, V v3, Constants.SwerveDrive.ModulePosition k4, V v4) {
        return new SwerveModuleMap<V>(Map.of(k1, v1, k2, v2, k3, v3, k4, v4));
    }

    /**
     * Returns the values from the map as a {@link List} in the same order as in the {@link Constants.SwerveDrive.ModulePosition} enum.<p>
     * Use instead of {@link #values() values} because that returns a {@link java.util.Collection}, which is not ordered.
     */
    public List<V> orderedValues() {
        ArrayList<V> list = new ArrayList<>();
        for (Constants.SwerveDrive.ModulePosition i : Constants.SwerveDrive.ModulePosition.values()) {
            list.add(get(i));
        }
        return list;
    }

    /**
     * Returns the values from the map as an {@code Array} in the same order as in the {@link Constants.SwerveDrive.ModulePosition} enum.<p>
     * Useful when a WPILib swerve function requires an array as input.
     *
     * @param array An array of the class to output an array of, e.g. {@code moduleTranslations.valuesArray(new Translation2d[0])}. Required because Java can't make an array of generics.
     */
    public V[] valuesArray(V[] array) {
        return orderedValues().toArray(array);
    }
}
