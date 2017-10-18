package Paquete2;
import java.awt.*;
import java.applet.*;
import java.awt.event.*;

public class AppletCalculadora extends Applet implements ItemListener {
	private static final long serialVersionUID = 1L;
	TextField t1;
	Button b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
	Button bpt, bCE, bC, bmas, bmenos, bigual, bpor, bdiv;
	String tx, ty, operacion;
	double x, y, resultado;
	boolean point_avalaible = true;
	boolean flagY = false;
	Panel p1, p2;
	
	public AppletCalculadora() {
		tx = "";
		ty = "";
		Panel p1 = new Panel();
		Panel p2 = new Panel();	
		p1.setLayout (new BorderLayout(8,1));
		p2.setLayout (new GridLayout(6,2,5,5));
		
		t1 = new TextField("Escriba aquí la operación");
		//boton 0
		b0 = new Button("0");
		b0.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
				ty = ty.concat("0");
				t1.setText(tx.concat(ty));
				}//fin if
				else {
				tx = tx.concat("0");
				t1.setText(tx);
				}//fin else
				
			}			
		}
		);
		//boton 1
		b1 = new Button("1");
		b1.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("1");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("1");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 2
		b2 = new Button("2");
		b2.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("2");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("2");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 3
		b3 = new Button("3");
		b3.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("3");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("3");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		//boton 4
		b4 = new Button("4");
		b4.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("4");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("4");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 5
		b5 = new Button("5");
		b5.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("5");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("5");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 6
		b6 = new Button("6");
		b6.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("6");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("6");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 7
		b7 = new Button("7");
		b7.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("7");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("7");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 8
		b8 = new Button("8");
		b8.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("8");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("8");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//boton 9
		b9 = new Button("9");
		b9.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if(flagY) {
					ty = ty.concat("9");
					t1.setText(tx.concat(ty));
					}//fin if
					else {
					tx = tx.concat("9");
					t1.setText(tx);
					}//fin else
			}			
		}
		);
		
		//punto decimal
		bpt = new Button(".");
		bpt.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if (point_avalaible) {
				if (!flagY) tx = tx.concat(".");
				if (flagY)  ty = ty.concat(".");				
				point_avalaible = false;
				t1.setText(tx + ty);
				}//fin if
			}			
		}
		);
		
		//boton +
		bmas = new Button("+");
		bmas.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if (operacion != "") t1.setText("Syntax Error");
				point_avalaible = true;
				flagY = true;
				x = Double.parseDouble(tx);
				tx = tx.concat(" + ");
				t1.setText(tx);
				operacion = "+";

			}			
		}
		);
		
		//boton -
		bmenos = new Button("-");
		bmenos.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if (operacion != "") t1.setText("Syntax Error");
				point_avalaible = true;
				flagY = true;
				x = Double.parseDouble(tx);
				tx = tx.concat(" - ");
				t1.setText(tx);
				operacion = "-";
			}			
		}
		);
		
		//boton *
		bpor = new Button("*");
		bpor.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if (operacion != "") t1.setText("Syntax Error");
				point_avalaible = true;
				flagY = true;
				x = Double.parseDouble(tx);
				tx = tx.concat(" * ");
				t1.setText(tx);
				operacion = "*";
			}			
		}
		);
		
		//boton /
		bdiv = new Button("/");
		bdiv.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				if (operacion != "") t1.setText("Syntax Error");
				point_avalaible = true;
				flagY = true;
				x = Double.parseDouble(tx);
				tx = tx.concat(" / ");
				t1.setText(tx);
				operacion = "/";
			}			
		}
		);
		
		//boton =
		bigual = new Button("=");
		bigual.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				y = Double.parseDouble(ty);
				System.out.println("Tx es " +tx );
				System.out.println("X es" +x);
				System.out.println("Ty es " +ty );
				System.out.println("Y es" +y);
				
				switch(operacion) {
				
				
				case "+":	resultado = x+y;
							t1.setText(String.valueOf(resultado));
							break;
					
				case "-":	resultado = x-y;
							t1.setText(String.valueOf(resultado));
							break;
				
				case "*":	resultado = x*y;
							t1.setText(String.valueOf(resultado));
							break;
				
				case "/":	resultado = x/y;
							if (y == 0) t1.setText("Math Error");
							else t1.setText(String.valueOf(resultado));
							break;
				}//fin sw case
				
				tx = "";
				ty = "";
				operacion = "";
				flagY = false;
			}			
		}
		);
		
		//boton C
		bC = new Button("C");
		bC.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {
				point_avalaible = true;
				tx = "";
				ty = "";
				operacion = "";
				t1.setText(tx);
				flagY = false;
				
			}			
		}
		);
		
		//boton CE
		bCE = new Button("CE");
		bCE.addActionListener( new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent b1) {

				t1.setText(" ");
				
			}			
		}
		);
		
		
		
		
		//agregar todo a la interfaz:
		add(p1);
		add(p2);
		p1.add(t1 , BorderLayout.NORTH);
		p2.add(b1);
		p2.add(b2);
		p2.add(b3);
		p2.add(b4);
		p2.add(b5);
		p2.add(b6);
		p2.add(b7);
		p2.add(b8);
		p2.add(b9);
		p2.add(bpt);
		p2.add(b0);
		p2.add(bCE);
		p2.add(bmas);
		p2.add(bmenos);
		p2.add(bigual);
		p2.add(bpor);
		p2.add(bdiv);		
		p2.add(bC);
		
	}
	@Override
	public void itemStateChanged(ItemEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	
		
	
		
	
}
