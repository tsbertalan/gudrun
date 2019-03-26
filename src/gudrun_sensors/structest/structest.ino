typedef struct {
    float x, y, z;
} Data;

void setup {
    Serial.begin(115200);
    Data to_send;
    data.x = 1;
    data.y = 2;
    data.z = 3;

    Serial.write((const char*)to_send, sizeof(to_send));

    Data to_receive;
    
}

void loop {
    ;
}