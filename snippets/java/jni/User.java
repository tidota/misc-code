import java.util.ArrayList;

public class User
{
    static
    {
        System.loadLibrary("wrapper");
    }

    public native int func(int val);

    public native User load();

    public static User createNode(User parent, String childName)
    {
      System.out.println("in createNode");
      return new User(parent, childName);
    }

    public static void main(String[] args)
    {
        User usr = new User(null, "root");
        System.out.println("calling func in the native code.");
        int buff = usr.func(10);
        System.out.println("Return val: " + buff);

        User root = usr.load();
        System.out.println("asdfsadf: " + root);
        System.out.println(root.name);
        /*
        System.out.println(root.children.get(0).name);
        System.out.println(root.children.get(1).name);
        System.out.println(root.children.get(1).children.get(0).name);
        System.out.println(root.children.get(1).children.get(1).name);
        System.out.println(root.children.get(2).name);
        */
    }

    public User(User newParent, String newName)
    {
      System.out.println("Constructor");
      this.parent = newParent;
      if (this.parent != null)
      {
        System.out.println("?");
        this.parent.children.add(this);
      }
      this.name = newName;
      System.out.println("!" + this.name);
    }

    public String name;
    public User parent;
    public ArrayList<User> children;
}
