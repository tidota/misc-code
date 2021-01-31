#!/usr/bin/ruby -w

# Some link
# https://www.tutorialspoint.com/ruby/ruby_syntax.htm

puts "Hello, Ruby!";

=begin
This is a comment section with more than one line.
This is also a comment.
=end

print <<EOF
line1
line2
line3
EOF

out =<<MIMIMI
oh!
oh2
oh3
MIMIMI
puts out;

print "print function";

class Foo
    CONS = 1000
    @@n_foos = 0
    def initialize(a, b, c)
        @vala = a
        @valb = b
        @valc = c
        @@n_foos += 1
    end

    def disp_data
        puts @vala 
        puts @valb
        puts @valc
        puts "n_foos = #@@n_foos"
        puts "cons = #{CONS}"
    end
end

f1 = Foo.new("a1", "b1", "c1")
puts "f1.disp_data"
f1.disp_data

f2 = Foo.new("a2", "b2", "c2")
puts "f2.disp_data"
f2.disp_data

array = [ "a", 1, 'a', 2, ]
array.each do |i|
    puts i
end


dict = {'red' => 0, 'green' => 1, 'blue' => 2, }
dict.each do |key, val|
    print key, " is ", val, "\n"
end

BEGIN {
puts "This is the beginning";
}

END {
puts "This is the ending";
}

