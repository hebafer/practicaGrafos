import java.util.ArrayList;

/**
 *
 */

public class AplicacionLogistica {


    public AplicacionLogistica(){
        RedLogistica red = new RedLogistica();
        System.out.println(red);

        /** camino mas corto **/
            Real distancia = new Real(0);
            ArrayList<String> camino;

            camino = red.caminoMasCorto("Madrid", "Amsterdam", distancia);
            imprimirCamino("Madrid", "Amsterdam" , camino, distancia);

            camino = red.caminoMasCorto("Lisboa", "Oslo", distancia);
            imprimirCamino("Lisboa", "Oslo" , camino, distancia);

            camino = red.caminoMasCorto("Dublin", "Atenas", distancia);
            imprimirCamino("Dublin", "Atenas" , camino, distancia);

        /** tramos criticos **/
            String[][] tramosCriticos;
            tramosCriticos = red.getTramosCriticos();
            imprimirTramosCriticos(tramosCriticos);

        /** almacenes criticos **/
            String[] almacenesCriticos;
            almacenesCriticos = red.getAlmacenesCriticos();
            imprimirAlmacenesCriticos(almacenesCriticos);

        /** almacen mas cercano al resto **/
            String[] almacenesMasCercanos;
            almacenesMasCercanos = red.getAlmacenesMasCercanos();
            imprimirAlmacenesMasCercanos(almacenesMasCercanos);

    }

    private void imprimirCamino(String origen, String destino, ArrayList<String> camino, Real distancia){
        System.out.println("-------------------------------------------------------------------");
        System.out.println("Ruta calculada para ir de " + origen + " a " + destino + ":");
        System.out.print("   ");
        for (int i=0;i<camino.size(); i++){
            System.out.print(camino.get(i) + "  ");
        }
        System.out.println();
        System.out.println("   La distancia total a recorrer es de " + distancia.getValor() + " kilometros");
        System.out.println();
    }

    private void imprimirTramosCriticos(String[][] tramosCriticos){
        System.out.println("-------------------------------------------------------------------");
        if (tramosCriticos[0].length==0)
            System.out.println("La red de transportes no tiene ningún tramo crítico");
        else{
            System.out.print("La red de transporte tiene " + tramosCriticos[0].length);
            if (tramosCriticos[0].length==1) System.out.println(" tramo critico:");
            else System.out.println(" tramos criticos:");
            for (int i=0; i< tramosCriticos[0].length; i++){
                System.out.println("  origen: " + tramosCriticos[0][i] + "; destino: " + tramosCriticos[1][i]);
            }
        }
        System.out.println();
    }

    private void imprimirAlmacenesCriticos(String[] almacenesCriticos){
        System.out.println("-------------------------------------------------------------------");
        if (almacenesCriticos.length==0)
            System.out.println("La red de transportes no tiene ningún almacén crítico");
        else{
            System.out.print("La red de transportes tiene "+ almacenesCriticos.length);
            if (almacenesCriticos.length==1) System.out.println(" almacen critico:");
            else System.out.println(" almacenes criticos:");
            for (int i=0; i< almacenesCriticos.length; i++){
                System.out.println("   " + almacenesCriticos[i]);
            }
        }

        System.out.println();
    }

    private void imprimirAlmacenesMasCercanos(String[] almacenesCentrales){
        System.out.println("-------------------------------------------------------------------");
        if (almacenesCentrales.length==0)
            System.out.println("La red de transportes no tiene ningún almacén Central");
        else{
            System.out.print("La red de transporte tiene "+ almacenesCentrales.length);
            if (almacenesCentrales.length==1) System.out.println(" almacen central:");
            else System.out.println("almacenes centrales");
            for (int i=0; i< almacenesCentrales.length; i++){
                System.out.println("   " + almacenesCentrales[i]);
            }
        }
    }

    public static void main(String[] args){
        new AplicacionLogistica();
    }

}
