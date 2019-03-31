public class User
{
    public native int func(int val);

    static
    {
        System.loadLibrary("sample");
    }

    public static void main(String[] args)
    {
        User usr = new User();
        System.out.println("calling func in the native code.");
        int buff = usr.func(10);
        System.out.println("Return val: " + buff);
    }
}
