#include <iostream>

#include <ctime>

void test1()
{
    std::cout << "print the current time" << std::endl;

    time_t rawtime;
    struct tm * timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    char buff[80];
    strftime (buff,80,"Now it's %I:%M%p, %B %d, %Y, %a.",timeinfo);

    std::cout << buff << std::endl << std::endl;
}

void test2()
{
    // https://www.cplusplus.com/reference/ctime/mktime/
    std::cout << "create a specific date" << std::endl;

    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    timeinfo->tm_sec = 0;
    timeinfo->tm_min = 30;
    timeinfo->tm_hour = 20;
    timeinfo->tm_mday = 22;
    timeinfo->tm_mon = 6 - 1;
    timeinfo->tm_year = 2021 - 1900;
    mktime(timeinfo);

    char buff[80];
    strftime (buff,80,"Now it's %I:%M%p, %B %d, %Y, %a.",timeinfo);

    std::cout << buff << std::endl << std::endl;
}

void test3()
{
    std::cout << "calculate the days between two dates" << std::endl;

    time_t rawtime;
    struct tm d1, d2;

    time(&rawtime);
    d1 = *localtime(&rawtime);

    d1.tm_sec = 0;
    d1.tm_min = 0;
    d1.tm_hour = 0;
    d1.tm_mday = 15;
    d1.tm_mon = 6 - 1;
    d1.tm_year = 2021 - 1900;
    d2 = d1;
    d2.tm_year = 2022 - 1900;

    double diff_time = difftime(mktime(&d2), mktime(&d1));
    std::cout << "diff time in sec: " << diff_time << std::endl;
    std::cout << "diff time in days: " << (diff_time / 3600 / 24) << std::endl;
}

int main()
{
    test1();

    test2();

    test3();

    return 0;
}
