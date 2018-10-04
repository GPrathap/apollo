#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>
#include <iostream>
using namespace std;
using namespace tensorflow;

int main()
{
    Session* session;
    Status status = NewSession(SessionOptions(), &session);
    if (!status.ok()) {
        cout << status.ToString() << endl;
        return 1;
    }
    cout << "Session successfully created." << endl;
}