package org.usfirst.frc.team2856.robot;

import java.util.LinkedList;
import java.util.TreeMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;

public abstract class AbstractInputManager {

	public static final int
			ATTACK3_AXIS_X = 0,
			ATTACK3_AXIS_Y = 1,
			ATTACK3_AXIS_Z = 2;
	
	public static final int
			XBOX_AXIS_LX       = 0,
			XBOX_AXIS_LY       = 1,
			XBOX_AXIS_LTRIGGER = 2,
			XBOX_AXIS_RTRIGGER = 3,
			XBOX_AXIS_RX       = 4,
			XBOX_AXIS_RY       = 5,
			
			XBOX_BUTTON_A      = 1,
			XBOX_BUTTON_B      = 2,
			XBOX_BUTOTN_X      = 3,
			XBOX_BUTTON_Y      = 4,
			XBOX_BUTTON_LBUMP  = 5,
			XBOX_BUTTON_RBUMP  = 6,
			XBOX_BUTTON_BACK   = 7,
			XBOX_BUTTON_START  = 8,
			XBOX_BUTTON_L3     = 9,
			XBOX_BUTTON_R3     = 10,
			
			XBOX_POV           = 0;
			
	private    Joystick[] joysticks;
	private       boolean newButton;
	private        double newAxis, newAxis2, newAxis3;
	private           int newPOV;
	
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Boolean>>> buttonMap;
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Double>>>  axisMap;
	private TreeMap<DualJoyAddr, LinkedList<BiAxisHandler>> biAxisMap;
	private TreeMap<TriJoyAddr, LinkedList<TriAxisHandler>> triAxisMap;
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Integer>>> povMap;
	
	public AbstractInputManager(Joystick... jsticks) {
		joysticks = jsticks;
		
		buttonMap = new TreeMap<>();
		axisMap = new TreeMap<>();
		biAxisMap = new TreeMap<>();
		triAxisMap = new TreeMap<>();
		povMap = new TreeMap<>();
	}
	
	public void update() {
		for(JoyAddr pair: buttonMap.keySet()) {
			newButton = joysticks[pair.ijoy].getRawButton(pair.ival);
			buttonMap.get(pair).forEach(handler -> {
				if(!handler.curr.equals(newButton)) {
					handler.curr = newButton;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(JoyAddr pair: axisMap.keySet()) {
			newAxis = joysticks[pair.ijoy].getRawAxis(pair.ival);
			axisMap.get(pair).forEach(handler -> {
				if(!handler.curr.equals(newAxis)) {
					handler.curr = newAxis;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(DualJoyAddr addr: biAxisMap.keySet()) {
			newAxis = joysticks[addr.ijoy].getRawAxis(addr.ival1);
			newAxis2 = joysticks[addr.ijoy].getRawAxis(addr.ival2);
			biAxisMap.get(addr).forEach(handler -> {
				if(handler.axis1 != newAxis
						|| handler.axis2 != newAxis2) {
					handler.axis1 = newAxis;
					handler.axis2 = newAxis2;
					handler.handler.accept(handler.axis1, handler.axis2);
				}
			});
		}
		
		for(TriJoyAddr addr: triAxisMap.keySet()) {
			newAxis = joysticks[addr.ijoy].getRawAxis(addr.ival1);
			newAxis2 = joysticks[addr.ijoy].getRawAxis(addr.ival2);
			newAxis3 = joysticks[addr.ijoy].getRawAxis(addr.ival3);
			
			triAxisMap.get(addr).forEach(handler -> {
				if(handler.axis1 != newAxis
						|| handler.axis2 != newAxis2
						|| handler.axis3 != newAxis3) {
					handler.axis1 = newAxis;
					handler.axis2 = newAxis2;
					handler.axis3 = newAxis3;
					handler.handler.accept(handler.axis1, handler.axis2,
							handler.axis3);
				}
			});
		}
		
		for(JoyAddr pair: povMap.keySet()) {
			newPOV = joysticks[pair.ijoy].getPOV(pair.ival);
			povMap.get(pair).forEach(handler -> {
				if(!handler.curr.equals(newPOV)) {
					handler.curr = newPOV;
					handler.handler.accept(handler.curr);
				}
			});
		}
	}
	
	public void addButtonAction(int joystick,
			int button, Consumer<Boolean> handler) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getRawButton(button), handler));
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeButtonAction(int joystick,
			int button, Consumer<Boolean> handler) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(false, handler));
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addAxisAction(int joystick,
			int axis, Consumer<Double> handler) {
		LinkedList<ValueHandler<Double>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getRawAxis(axis), handler));
		axisMap.merge(new JoyAddr(joystick, axis), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeAxisAction(int joystick,
			int axis, Consumer<Double> handler) {
		LinkedList<ValueHandler<Double>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(0.0, handler));
		axisMap.merge(new JoyAddr(joystick, axis), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addBiAxisAction(int joystick,
			int axis1, int axis2, BiConsumer<Double, Double> handler) {
		LinkedList<BiAxisHandler> list = new LinkedList<>();
		list.addFirst(new BiAxisHandler(
				joysticks[joystick].getRawAxis(axis1),
				joysticks[joystick].getRawAxis(axis2), handler));
		biAxisMap.merge(new DualJoyAddr(joystick, axis1, axis2), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeBiAxisAction(int joystick,
			int axis1, int axis2, BiConsumer<Double, Double> handler) {
		LinkedList<BiAxisHandler> list = new LinkedList<>();
		list.addFirst(new BiAxisHandler(0.0, 0.0, handler));
		biAxisMap.merge(new DualJoyAddr(joystick, axis1, axis2), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addTriAxisAction(int joystick,
			int axis1, int axis2, int axis3,
			TriConsumer<Double, Double, Double> handler) {
		LinkedList<TriAxisHandler> list = new LinkedList<>();
		list.addFirst(new TriAxisHandler(
				joysticks[joystick].getRawAxis(axis1),
				joysticks[joystick].getRawAxis(axis2),
				joysticks[joystick].getRawAxis(axis3), handler));
		triAxisMap.merge(new TriJoyAddr(joystick, axis1, axis2, axis3), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeTriAxisAction(int joystick,
			int axis1, int axis2, int axis3,
			TriConsumer<Double, Double, Double> handler) {
		LinkedList<TriAxisHandler> list = new LinkedList<>();
		list.addFirst(new TriAxisHandler(0.0, 0.0, 0.0, handler));
		triAxisMap.merge(new TriJoyAddr(joystick, axis1, axis2, axis3), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addPOVAction(int joystick,
			int pov, Consumer<Integer> handler) {
		LinkedList<ValueHandler<Integer>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getPOV(pov), handler));
		povMap.merge(new JoyAddr(joystick, pov), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removePOVAction(int joystick,
			int pov, Consumer<Integer> handler) {
		LinkedList<ValueHandler<Integer>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(0, handler));
		povMap.merge(new JoyAddr(joystick, pov), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	private static class JoyAddr implements Comparable<JoyAddr> {
		
		public final int ijoy;
		public final int ival;
		
		public JoyAddr(int ijoy, int ivalue) {
			this.ijoy = ijoy;
			this.ival = ivalue;
		}
		
		@Override
		public int compareTo(JoyAddr o) {
			return o == null ? 1
				 : ijoy != o.ijoy ? (ijoy > o.ijoy ? 1 : -1)
				 : ival != o.ival ? (ival > o.ival ? 1 : -1)
				 : 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o != null) && (o instanceof JoyAddr)
					&& (compareTo((JoyAddr) o) == 0);
		}
		
	}
	
	private static class DualJoyAddr implements Comparable<DualJoyAddr> {

		public final int ijoy;
		public final int ival1;
		public final int ival2;
		
		public DualJoyAddr(int ijoy, int ival1, int ival2) {
			this.ijoy = ijoy;
			this.ival1 = ival1;
			this.ival2 = ival2;
		}
		
		@Override
		public int compareTo(DualJoyAddr o) {
			return o == null ? 1
				 : ijoy  != o.ijoy  ? (ijoy  > o.ijoy  ? 1 : -1)
				 : ival1 != o.ival1 ? (ival1 > o.ival1 ? 1 : -1)
				 : ival2 != o.ival2 ? (ival2 > o.ival2 ? 1 : -1)
				 : 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o != null) && (o instanceof DualJoyAddr)
					&& (compareTo((DualJoyAddr) o) == 0);
		}
		
	}
	
	private static class TriJoyAddr implements Comparable<TriJoyAddr> {

		public final int ijoy;
		public final int ival1;
		public final int ival2;
		public final int ival3;
		
		public TriJoyAddr(int ijoy, int ival1, int ival2, int ival3) {
			this.ijoy = ijoy;
			this.ival1 = ival1;
			this.ival2 = ival2;
			this.ival3 = ival3;
		}
		
		@Override
		public int compareTo(TriJoyAddr o) {
			return o == null ? 1
				 : ijoy  != o.ijoy  ? (ijoy  > o.ijoy  ? 1 : -1)
				 : ival1 != o.ival1 ? (ival1 > o.ival1 ? 1 : -1)
				 : ival2 != o.ival2 ? (ival2 > o.ival2 ? 1 : -1)
				 : ival3 != o.ival3 ? (ival3 > o.ival3 ? 1 : -1)
				 : 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o != null) && (o instanceof TriJoyAddr)
					&& (compareTo((TriJoyAddr) o) == 0);
		}
		
	}
	
	private static class ValueHandler<T> {
		
		public T curr;
		public final Consumer<T> handler;
		
		public ValueHandler(T curr, Consumer<T> handler) {
			this.curr = curr;
			this.handler = handler;
		}
		
		@Override
		public boolean equals(Object o) {
			if(o == null || !(o instanceof ValueHandler<?>)) {
				return false;
			}
			
			return (handler == ((ValueHandler<?>)o).handler);
		}
		
	}
	
	private static class BiAxisHandler {
		
		public double axis1, axis2;
		public final BiConsumer<Double, Double> handler;
		
		public BiAxisHandler(double axis1, double axis2,
				BiConsumer<Double, Double> handler) {
			this.axis1 = axis1;
			this.axis2 = axis2;
			this.handler = handler;
		}
		
		@Override
		public boolean equals(Object o) {
			if(o == null || !(o instanceof BiAxisHandler)) {
				return false;
			}
			
			return (handler == ((BiAxisHandler)o).handler);
		}
		
	}
	
	private static class TriAxisHandler {
		
		public double axis1, axis2, axis3;
		public final TriConsumer<Double, Double, Double> handler;
		
		public TriAxisHandler(double axis1, double axis2, double axis3,
				TriConsumer<Double, Double, Double> handler) {
			this.axis1 = axis1;
			this.axis2 = axis2;
			this.axis3 = axis3;
			this.handler = handler;
		}
		
		@Override
		public boolean equals(Object o) {
			if(o == null || !(o instanceof TriAxisHandler)) {
				return false;
			}
			
			return (handler == ((TriAxisHandler)o).handler);
		}
		
	}
	
	@FunctionalInterface
	private static interface TriConsumer<T, U, V> {
		
		void accept(T t, U u, V v);
		
		default TriConsumer<T, U, V> andThen(
				TriConsumer<? super T, ? super U, ? super V> after) {
			return (t_, u_, v_) -> {
				accept(t_, u_, v_);
				after.accept(t_, u_, v_);
			};
		}
		
	}
	
}
