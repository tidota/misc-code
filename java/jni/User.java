import java.util.ArrayList;

class Node
{
    public Node(Node newParent, String newName)
    {
      this.children = new ArrayList<Node>();
      this.parent = newParent;
      if (this.parent != null)
      {
        this.parent.children.add(this);
      }
      this.name = newName;
    }

    public String name;
    public Node parent;
    public ArrayList<Node> children;
}
public class User
{

    static
    {
        System.loadLibrary("DBWrapper");
    }

    public native Node load();

    public static Node createNode(Node parent, String childName)
    {
      return new Node(parent, childName);
    }

    public static void main(String[] args)
    {
        User usr = new User();

        Node root = usr.load();
        System.out.println("root: " + root);
        System.out.println(root.children.get(0).name);
        System.out.println(root.children.get(1).name);
        System.out.println(root.children.get(1).children.get(0).name);
        System.out.println(root.children.get(1).children.get(1).name);
        System.out.println(root.children.get(2).name);
    }

}
