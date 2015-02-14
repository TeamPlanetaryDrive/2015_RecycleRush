package org.usfirst.frc.team2856.robot;

public abstract class AbstractInputManager {

	private static class JoyAddr implements Comparable<JoyAddr> {
		public final int joy, inp;
		
		public JoyAddr(int joy, int inp) {
			this.joy = joy;
			this.inp = inp;
		}
		
		@Override
		public int compareTo(JoyAddr o) {
			return (o == null) ? 1
				: (joy != o.joy) ? (joy > o.joy ? 1 : -1)
				: (inp != o.inp) ? (inp > o.inp ? 1 : -1)
				: 0;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o instanceof JoyAddr)
					&& compareTo((JoyAddr) o) == 0;
		}
	}
	
	private static class ButtonHandler {
		public final BooleanConsumer handler;
		public boolean curr;
		public boolean cont;
		public Runnable deferred;
		
		public ButtonHandler(BooleanConsumer handler, boolean curr,
				boolean cont) {
			this.handler = handler;
			this.curr = curr;
			this.cont = cont;
			this.deferred = null;
		}
		
		public ButtonHandler(Runnable run, boolean curr, boolean cont) {
			this(value -> { if(value) run.run(); }, curr, cont);
			this.deferred = run;
		}
		
		@Override
		public boolean equals(Object o) {
			return (o instanceof ButtonHandler && o != null)
				&& (deferred != null ?
						(deferred == ((ButtonHandler) o).deferred) :
						(handler == ((ButtonHandler) o).handler));
		}
	}
	
	@FunctionalInterface
	public static interface BooleanConsumer {
		public void accept(boolean b);
	}
}
