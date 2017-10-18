package Paquete2;
import java.awt.*;

import java.applet.*;
import java.awt.event.*;

public class Interfaz1 extends Applet implements ItemListener {
	private static final long serialVersionUID = 1402473259358095622L;
	Choice ch1, ch2;
	public Interfaz1() {
		ch1 = new Choice();
		ch2 = new Choice();
		ch1.add("primero");
		ch1.add("segundo");
		ch1.add("tercero");
		ch1.add("cuarto");
		ch1.add("quinto");
		ch2.add("uno");
		ch2.add("dos");
		ch2.add("tres");
		ch2.add("cuatro");
		ch2.add("cinco");
		add(ch1);
		add(ch2);
		ch1.addItemListener(this);
		ch2.addItemListener(this);				
	}

	public void itemStateChanged(ItemEvent ie) {
	repaint();	
	}
	
	public void paint(Graphics g) {
		String msg = "Primer Menú = ";
		msg += ch1.getSelectedItem();
		g.drawString(msg, 0, 100);
		msg = "Segundo Menú = ";
		msg += ch2.getSelectedItem();
		g.drawString(msg, 0, 140);
	}
}
