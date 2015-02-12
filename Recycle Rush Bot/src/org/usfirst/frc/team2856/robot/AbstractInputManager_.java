package org.usfirst.frc.team2856.robot;

import java.util.LinkedList;
import java.util.TreeMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;

public abstract class AbstractInputManager_ {

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
	
	public AbstractInputManager_(Joystick... jsticks) {
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
				if(handler.cont || !handler.curr.equals(newButton)) {
					handler.curr = newButton;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(JoyAddr pair: axisMap.keySet()) {
			newAxis = joysticks[pair.ijoy].getRawAxis(pair.ival);
			axisMap.get(pair).forEach(handler -> {
				if(handler.cont || !handler.curr.equals(newAxis)) {
					handler.curr = newAxis;
					handler.handler.accept(handler.curr);
				}
			});
		}
		
		for(DualJoyAddr addr: biAxisMap.keySet()) {
			newAxis = joysticks[addr.addr1.ijoy].getRawAxis(addr.addr1.ival);
			newAxis2 = joysticks[addr.addr2.ijoy].getRawAxis(addr.addr2.ival);
			biAxisMap.get(addr).forEach(handler -> {
				if(handler.cont || handler.axis1 != newAxis
						|| handler.axis2 != newAxis2) {
					handler.axis1 = newAxis;
					handler.axis2 = newAxis2;
					handler.handler.accept(handler.axis1, handler.axis2);
				}
			});
		}
		
		for(TriJoyAddr addr: triAxisMap.keySet()) {
			newAxis = joysticks[addr.addr1.ijoy].getRawAxis(addr.addr1.ival);
			newAxis2 = joysticks[addr.addr2.ijoy].getRawAxis(addr.addr2.ival);
			newAxis3 = joysticks[addr.addr3.ijoy].getRawAxis(addr.addr3.ival);
			
			triAxisMap.get(addr).forEach(handler -> {
				if(handler.cont || handler.axis1 != newAxis
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
				if(handler.cont || !handler.curr.equals(newPOV)) {
					handler.curr = newPOV;
					handler.handler.accept(handler.curr);
				}
			});
		}
	}
	
	public void addButtonAction(int joystick,
			int button, Consumer<Boolean> handler, boolean continuous) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getRawButton(button), handler,
					continuous));
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeButtonAction(int joystick,
			int button, Consumer<Boolean> handler) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(false, handler, false));
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addButtonAction(int joystick,
			int button, Runnable handler, boolean continuous) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getRawButton(button), value -> {
					if(value) handler.run();
				}, continuous));
		list.getFirst().buttonHandler = handler;
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeButtonAction(int joystick,
			int button, Runnable handler) {
		LinkedList<ValueHandler<Boolean>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(false, value -> {}, false));
		list.getFirst().buttonHandler = handler;
		buttonMap.merge(new JoyAddr(joystick, button), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addAxisAction(int joystick,
			int axis, Consumer<Double> handler, boolean continuous) {
		LinkedList<ValueHandler<Double>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getRawAxis(axis), handler, continuous));
		axisMap.merge(new JoyAddr(joystick, axis), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeAxisAction(int joystick,
			int axis, Consumer<Double> handler) {
		LinkedList<ValueHandler<Double>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(0.0, handler, false));
		axisMap.merge(new JoyAddr(joystick, axis), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addBiAxisAction(int joystick1, int axis1,
			int joystick2, int axis2, BiConsumer<Double, Double> handler,
			boolean continuous) {
		LinkedList<BiAxisHandler> list = new LinkedList<>();
		list.addFirst(new BiAxisHandler(
				joysticks[joystick1].getRawAxis(axis1),
				joysticks[joystick2].getRawAxis(axis2), handler, continuous));
		biAxisMap.merge(new DualJoyAddr(new JoyAddr(joystick1, axis1),
										new JoyAddr(joystick2, axis2)), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeBiAxisAction(int joystick1, int axis1,
			int joystick2, int axis2, BiConsumer<Double, Double> handler) {
		LinkedList<BiAxisHandler> list = new LinkedList<>();
		list.addFirst(new BiAxisHandler(0.0, 0.0, handler, false));
		biAxisMap.merge(new DualJoyAddr(new JoyAddr(joystick1, axis1),
										new JoyAddr(joystick2, axis2)), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addTriAxisAction(int joystick1, int axis1,
			int joystick2, int axis2, int joystick3, int axis3,
			TriConsumer<Double, Double, Double> handler, boolean continuous) {
		LinkedList<TriAxisHandler> list = new LinkedList<>();
		list.addFirst(new TriAxisHandler(
				joysticks[joystick1].getRawAxis(axis1),
				joysticks[joystick2].getRawAxis(axis2),
				joysticks[joystick3].getRawAxis(axis3), handler, continuous));
		triAxisMap.merge(new TriJoyAddr(new JoyAddr(joystick1, axis1),
										new JoyAddr(joystick2, axis2),
										new JoyAddr(joystick3, axis3)), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removeTriAxisAction(int joystick1, int axis1,
			int joystick2, int axis2, int joystick3, int axis3,
			TriConsumer<Double, Double, Double> handler) {
		LinkedList<TriAxisHandler> list = new LinkedList<>();
		list.addFirst(new TriAxisHandler(0.0, 0.0, 0.0, handler, false));
		triAxisMap.merge(new TriJoyAddr(new JoyAddr(joystick1, axis1),
										new JoyAddr(joystick2, axis2),
										new JoyAddr(joystick3, axis3)), list,
				(v0, v1) -> {
					v0.remove(v1.getFirst());
					return v0.isEmpty() ? null : v0;
				});
	}
	
	public void addPOVAction(int joystick,
			int pov, Consumer<Integer> handler, boolean continuous) {
		LinkedList<ValueHandler<Integer>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(
				joysticks[joystick].getPOV(pov), handler, continuous));
		povMap.merge(new JoyAddr(joystick, pov), list,
				(v0, v1) -> {
					v0.addFirst(v1.getFirst());
					return v0;
				});
	}
	
	public void removePOVAction(int joystick,
			int pov, Consumer<Integer> handler) {
		LinkedList<ValueHandler<Integer>> list = new LinkedList<>();
		list.addFirst(new ValueHandler<>(0, handler, false));
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

		public final JoyAddr addr1, addr2;
		
		public DualJoyAddr(JoyAddr addr1, JoyAddr addr2) {
			this.addr1 = addr1;
			this.addr2 = addr2;
		}
		
		@Override
		public int compareTo(DualJoyAddr o) {
			return o == null ? 1
				 : !addr1.equals(o.addr1) ? (addr1.compareTo(o.addr1))
				 : !addr2.equals(o.addr2) ? (addr2.compareTo(o.addr2))
				 : 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o != null) && (o instanceof DualJoyAddr)
					&& (compareTo((DualJoyAddr) o) == 0);
		}
		
	}
	
	private static class TriJoyAddr implements Comparable<TriJoyAddr> {

		public final JoyAddr addr1, addr2, addr3;
		
		public TriJoyAddr(JoyAddr addr1, JoyAddr addr2, JoyAddr addr3) {
			this.addr1 = addr1;
			this.addr2 = addr2;
			this.addr3 = addr3;
		}
		
		@Override
		public int compareTo(TriJoyAddr o) {
			return o == null ? 1
				 : !addr1.equals(o.addr1) ? (addr1.compareTo(o.addr1))
				 : !addr2.equals(o.addr2) ? (addr2.compareTo(o.addr2))
				 : !addr3.equals(o.addr3) ? (addr3.compareTo(o.addr3))
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
		public boolean cont;
		public Runnable buttonHandler;
		
		public ValueHandler(T curr, Consumer<T> handler, boolean cont) {
			this.curr = curr;
			this.handler = handler;
			this.cont = cont;
			this.buttonHandler = null;
		}
		
		@Override
		public boolean equals(Object o) {
			if(o == null || !(o instanceof ValueHandler<?>)) {
				return false;
			}
			
			return (buttonHandler != null && buttonHandler
					== ((ValueHandler<?>)o).buttonHandler) ||
					(handler == ((ValueHandler<?>)o).handler);
		}
		
	}
	
	private static class BiAxisHandler {
		
		public double axis1, axis2;
		public final BiConsumer<Double, Double> handler;
		public boolean cont;
		
		public BiAxisHandler(double axis1, double axis2,
				BiConsumer<Double, Double> handler, boolean cont) {
			this.axis1 = axis1;
			this.axis2 = axis2;
			this.handler = handler;
			this.cont = cont;
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
		public boolean cont;
		
		public TriAxisHandler(double axis1, double axis2, double axis3,
				TriConsumer<Double, Double, Double> handler, boolean cont) {
			this.axis1 = axis1;
			this.axis2 = axis2;
			this.axis3 = axis3;
			this.handler = handler;
			this.cont = cont;
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
	public static interface TriConsumer<T, U, V> {
		
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
