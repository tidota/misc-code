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

BEGIN {
puts "This is the beginning";
}

END {
puts "This is the ending";
}

