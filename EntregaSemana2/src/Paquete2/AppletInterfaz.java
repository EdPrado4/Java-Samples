package Paquete2;
import java.awt.*;
import java.applet.*;
import java.awt.event.*;



public class AppletInterfaz extends Applet implements ItemListener {
	private static final long serialVersionUID = 1L;
	List lst1;
	String opcion = ""; 
	
	public AppletInterfaz() {
		lst1 = new List();
		lst1.add("Cyan");
		lst1.add("Magenta");
		lst1.add("Gris");
		lst1.add("Negro");
		lst1.add("Blanco");
		add(lst1);
		lst1.addItemListener(this);
	}
	
	public void itemStateChanged(ItemEvent e) {
		repaint();
	}

	public void paint(Graphics g) {
		
		opcion = lst1.getSelectedItem();
		System.out.println("opcion es "+opcion);
		
		switch (opcion) {
		
		case "Cyan":
			g.setColor(Color.cyan);
			g.fillRect(getX(), getY(), getWidth(), getHeight());
			break;
			
		case "Magenta":
			g.setColor(Color.MAGENTA);
			g.fillRect(getX(), getY(), getWidth(), getHeight());
			break;
		
		case "Gris":
			g.setColor(Color.gray);
			g.fillRect(getX(), getY(), getWidth(), getHeight());
			break;
		
		case "Negro":
			g.setColor(Color.BLACK);
			g.fillRect(getX(), getY(), getWidth(), getHeight());
			break;
		
		case "Blanco":
			g.setColor(Color.white);
			g.fillRect(getX(), getY(), getWidth(), getHeight());
			break;
		
		default:
			break;
		}//fin sw case
		
	}
	
	
}
