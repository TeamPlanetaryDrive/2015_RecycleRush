package org.usfirst.frc.team2856.robot;

import java.util.LinkedList;
import java.util.TreeMap;
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
	private        double newAxis;
	private           int newPOV;
	
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Boolean>>> buttonMap;
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Double>>>  axisMap;
	private TreeMap<JoyAddr, LinkedList<ValueHandler<Integer>>> povMap;
	
	public AbstractInputManager(Joystick... jsticks) {
		joysticks = jsticks;
		
		buttonMap = new TreeMap<>();
		axisMap = new TreeMap<>();
		povMap = new TreeMap<>();
	}
	
	public void update() {
		for(JoyAddr pair: buttonMap.keySet()) {
			newButton = joysticks[pair.ijoy].getRawButton(pair.ivalue);
			buttonMap.get(pair).forEach(handler -> {
				if(!handler.curr.equals(newButton)) {
					handler.curr = newButton;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(JoyAddr pair: axisMap.keySet()) {
			newAxis = joysticks[pair.ijoy].getRawAxis(pair.ivalue);
			axisMap.get(pair).forEach(handler -> {
				if(!handler.curr.equals(newAxis)) {
					handler.curr = newAxis;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(JoyAddr pair: povMap.keySet()) {
			newPOV = joysticks[pair.ijoy].getPOV(pair.ivalue);
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
		public final int ivalue;
		
		public JoyAddr(int ijoy, int ivalue) {
			this.ijoy = ijoy;
			this.ivalue = ivalue;
		}
		
		@Override
		public int compareTo(JoyAddr o) {
			return o == null ? 1
				 : ijoy   != o.ijoy   ? (ijoy   > o.ijoy   ? 1 : -1)
				 : ivalue != o.ivalue ? (ivalue > o.ivalue ? 1 : -1)
				 : 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o != null) && (o instanceof JoyAddr)
					&& (compareTo((JoyAddr) o) == 0);
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
	
}
