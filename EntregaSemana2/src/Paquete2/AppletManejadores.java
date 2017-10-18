package Paquete2;
import java.awt.*;
import java.applet.*;
import java.awt.event.*;


public class AppletManejadores extends Applet implements ItemListener {
	private static final long serialVersionUID = 1L;
	Label labelx, labely, labelres;
	TextField tx, ty, tres;
	double resultado;
	Button btnmas, btnmenos, btnpor, btndiv, btnrootx, btnrooty, btnxexpy, btnyexpx, btnxmody;
	public AppletManejadores() {
		//organizar los elementos
		setLayout (new GridLayout(5,4,3,3));
		//labels
		labelx = new Label("X");		
		labely = new Label("Y");		
		labelres = new Label("Resultado");
		
		
		//TextFields
		tx = new TextField("Variable x", 8);
		ty = new TextField("Variable y", 8);
		tres = new TextField ("Resultado", 8);
		tres.setEditable(false);
		
		//botones
		btnmas = new Button("+");
		btnmas.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Double.parseDouble(tx.getText()) + Double.parseDouble(ty.getText());
				tres.setText(""+resultado);
			}			
		}
		);
		btnmenos = new Button("-");
		btnmenos.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Double.parseDouble(tx.getText()) - Double.parseDouble(ty.getText());
				tres.setText(""+resultado);
			}			
		}
		);
		btnpor = new Button("*");
		btnpor.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Double.parseDouble(tx.getText()) * Double.parseDouble(ty.getText());
				tres.setText(""+resultado);
			}			
		}
		);
		btndiv = new Button("/");
		btndiv.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Double.parseDouble(tx.getText()) / Double.parseDouble(ty.getText());
				tres.setText(""+resultado);
			}			
		}
		);
		btnrootx = new Button("raíz x");
		btnrootx.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Math.sqrt(Double.parseDouble(tx.getText()));
				tres.setText(""+resultado);
			}			
		}
		);
		btnrooty = new Button("raíz y");
		btnrooty.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Math.sqrt(Double.parseDouble(ty.getText()));
				tres.setText(""+resultado);
			}			
		}
		);
		btnxexpy = new Button("x a la y");
		btnxexpy.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Math.pow(Double.parseDouble(tx.getText()), Double.parseDouble(ty.getText()));
				tres.setText(""+resultado);
			}			
		}
		);
		btnyexpx = new Button("y a la x");
		btnyexpx.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Math.pow(Double.parseDouble(ty.getText()), Double.parseDouble(tx.getText()));
				tres.setText(""+resultado);
			}			
		}
		);
		btnxmody = new Button("x mod y");
		btnxmody.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				resultado = Double.parseDouble(tx.getText()) % Double.parseDouble(ty.getText());
				tres.setText(""+resultado);
			}			
		}
		);
		
		//agregar todo a la interfaz
		add(labelx);
		add(labely);
		add(labelres);
		
		add(tx);
		add(ty);
		add(tres);
		
		add(btnmas);
		add(btnmenos);
		add(btnpor);
		add(btndiv);
		add(btnrootx);
		add(btnrooty);
		add(btnxexpy);
		add(btnyexpx);
		add(btnxmody);
		
	}
	
	
	public void itemStateChanged(ItemEvent e) {
		// TODO Auto-generated method stub
		
	}
	
}//fin AppletManejadores
