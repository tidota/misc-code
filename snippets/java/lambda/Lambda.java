import java.util.function.*;

public class Lambda
{
	public static void main(String[] args) {
		Supplier<String> f = () -> { return "test"; };
		System.out.println(f.get());
	}
}