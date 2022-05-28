package frc.team449.util

import edu.wpi.first.hal.SimBoolean
import edu.wpi.first.hal.SimDouble
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

/** A property with a custom getter but a backing field of a different type */
fun <B, F> wrappedProp(backing: B, get: (B) -> F): ReadOnlyProperty<Any?, F> {
  return object : ReadOnlyProperty<Any?, F> {
    override operator fun getValue(thisRef: Any?, property: KProperty<*>) = get(backing)
  }
}

/** A property with a custom getter and setter but a backing field of a different type */
fun <B, F> wrappedProp(backing: B, get: (B) -> F, set: (B, F) -> Unit): ReadWriteProperty<Any?, F> {
  return object : ReadWriteProperty<Any?, F> {
    override operator fun getValue(thisRef: Any?, property: KProperty<*>) = get(backing)

    override operator fun setValue(thisRef: Any?, property: KProperty<*>, value: F) = set(backing, value)
  }
}

/** A delegated property of type Double backed by a [SimDouble] */
fun simDoubleProp(sim: SimDouble): ReadWriteProperty<Any?, Double> = wrappedProp(sim, SimDouble::get, SimDouble::set)

/** A delegated property of type Boolean backed by a [SimBoolean] */
fun simBooleanProp(sim: SimBoolean): ReadWriteProperty<Any?, Boolean> = wrappedProp(sim, SimBoolean::get, SimBoolean::set)
