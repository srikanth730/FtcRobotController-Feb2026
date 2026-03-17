public class Test {


 // Field (attribute)
    String color = "Red";

    // Method (behavior)
    void drive() {
        System.out.println("The " + color + " car is driving!");
    }

    // Main method, the program's entry point
    public static void main(String[] args) {
        // Create an object (instance) of the Car class
        Car myCar = new Car();

        // Access the object's attribute
        System.out.println("My car's color is: " + myCar.color);

        // Access the object's method
        myCar.drive();
    
}
}
