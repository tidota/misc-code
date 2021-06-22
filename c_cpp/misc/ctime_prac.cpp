#include <iostream>

#include <ctime>

void print(struct tm * timeinfo)
{
    char buff[80];
    strftime (buff,80,"Now it's %I:%M%p, %B %d, %Y, %a.",timeinfo);
    std::cout << buff << std::endl << std::endl;
}

void test1()
{
    std::cout << "print the current time" << std::endl;

    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    print(timeinfo);
}

void test2()
{
    // https://www.cplusplus.com/reference/ctime/mktime/
    std::cout << "create a specific date" << std::endl;

    struct tm timeinfo = {.tm_mday = 1};

    timeinfo.tm_sec = 0;
    timeinfo.tm_min = 30;
    timeinfo.tm_hour = 20;
    timeinfo.tm_mday = 22;
    timeinfo.tm_mon = 6 - 1;
    timeinfo.tm_year = 2021 - 1900;
    mktime(&timeinfo);

    print(&timeinfo);
}

void test3()
{
    std::cout << "calculate the days between two dates" << std::endl;

    struct tm d1 = {.tm_mday=1}, d2 = {.tm_mday=1};

    d1.tm_mday = 15;
    d1.tm_mon = 6 - 1;
    d1.tm_year = 2021 - 1900;
    d2 = d1;
    d2.tm_year = 2022 - 1900;

    double diff_time = difftime(mktime(&d2), mktime(&d1));
    std::cout << "diff time in sec: " << diff_time << std::endl;
    std::cout << "diff time in days: " << (diff_time / 3600 / 24) << std::endl;

    std::cout << std::endl;
}

void test4()
{
    std::cout << "calcualte a date X days ahead of the other" << std::endl;

    std::cout << "orig:";

    struct tm orig = {.tm_mday=1};
    orig.tm_mday = 15;
    orig.tm_mon = 6 - 1;
    orig.tm_year = 2021 - 1900;

    print(&orig);

    time_t orig_time = mktime(&orig);

    time_t next_time = orig_time + 60 * 24 * 3600;
    struct tm next = *localtime(&next_time);
    std::cout << "60 days later:";
    print(&next);

    time_t prev_time = orig_time - 365 * 24 * 3600;
    struct tm prev = *localtime(&prev_time);
    std::cout << "365 days before:";
    print(&prev);
}

int main()
{
    test1();

    test2();

    test3();

    test4();

    return 0;
}
