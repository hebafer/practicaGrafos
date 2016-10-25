
/**
 * Modela un objeto que contiene una propiedad de tipo double
 */

public class Real {

    private double valor;

    /**
     * Crea un objeto de tipo Real con el valor indicado como parametro
     * @param valor valor double a asignar al objeto Real
     */
    public Real(double valor){
        this.valor = valor;
    }

    /**
     * Crea un objeto de tipo Real y lo inicializa con el valor que Java asigna por defecto a los tipos double
     */
    public Real(){
    }

    /**
     * Devuelve el valor de la propiedad double
     * @return valor de la propiedad double
     */
    public double getValor(){
        return this.valor;
    }

    /**
     * Modifica el valor de la propiedad double con el valor que se pasa como parametro
     * @param valor valor con el que modificar la propiedad double
     */
    public void setValor(double valor){
        this.valor = valor;
    }

    /**
     * Devuelve la representacion en formato String del objeto Real
     * @return representacion en formato String del objeto Real
     */

    public String toString(){
        return "" + this.valor;
    }
}
