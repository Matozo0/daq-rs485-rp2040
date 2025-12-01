#include <SoftwareSerial.h>
#include <vector>

class SerialRS485 : public SerialPIO
{
private:
    int _txPin;
    int _rxPin;
    int _deRePin;
    int _baudRate;
    enum SensorType {
        NONE = 0,
        ACCELEROMETER = 1,
        STRAIN_GAUGE = 2,
        LASER_DISTANCE = 3,        
    } _sensorType;

    void sendByte(uint8_t byte)
    {
        uint32_t _bitDelay = 1000000 / _baudRate;
        bool _highSpeed = _baudRate > 500000;
        digitalWrite(_txPin, LOW);
        if (_highSpeed)
            delayMicroseconds(1);
        else
            delayMicroseconds(_bitDelay);

        for (int i = 0; i < 8; i++)
        {
            digitalWrite(_txPin, (byte >> i) & 0x01);
            if (_highSpeed)
                delayMicroseconds(1);
            else
                delayMicroseconds(_bitDelay);
        }

        digitalWrite(_txPin, HIGH);
        if (_highSpeed)
            delayMicroseconds(1);
        else
            delayMicroseconds(_bitDelay);
    }

public:
    SerialRS485(int txPin, int rxPin, int deRePin, size_t fifoSize = 4096)
        : SerialPIO(NOPIN, rxPin, fifoSize), _txPin(txPin), _rxPin(rxPin), _deRePin(deRePin)
    {
    }

    void begin(unsigned long baudRate = 115200) override
    {
        _baudRate = baudRate;
        pinMode(_txPin, OUTPUT);
        pinMode(_rxPin, INPUT_PULLUP);
        pinMode(_deRePin, OUTPUT);
        digitalWrite(_txPin, HIGH);
        digitalWrite(_deRePin, LOW);
        SerialPIO::begin(baudRate);
    }

    void sendCommand(const uint8_t *buffer, size_t length)
    {
        digitalWrite(_deRePin, HIGH);
        delayMicroseconds(10);

        for (size_t i = 0; i < length; i++)
        {
            sendByte(buffer[i]);
        }

        digitalWrite(_deRePin, LOW);
        delayMicroseconds(10);
    }

    void setBaudRate(int baudRate)
    {
        if (_baudRate != baudRate)
        {
            _baudRate = baudRate;
            SerialPIO::end();
            SerialPIO::begin(baudRate);
            pinMode(_rxPin, INPUT_PULLUP);
        }
    }

    SensorType getSensorType()
    {
        return _sensorType;
    }

    bool setSensorType(SensorType type)
    {
        if (_sensorType == type)
            return false;

        _sensorType = type;
        switch (type)
        {
            case ACCELEROMETER:
                setBaudRate(1000000);
                break;
            case STRAIN_GAUGE:
                setBaudRate(115200);
                break;
            case LASER_DISTANCE:
                setBaudRate(115200);
                break;
            default:
                return false; 
        }
        return true;
    }
};

class RS485Manager
{
    private:
    std::vector<SerialRS485*> _ports;

    public:
    RS485Manager() = default;

    int addPort(int txPin, int rxPin, int deRePin, size_t fifoSize = 4096)
    {
        if (_ports.size() >= 8)
            return -1;

        SerialRS485* newPort = new SerialRS485(txPin, rxPin, deRePin, fifoSize);
        _ports.push_back(newPort);
        return _ports.size() - 1;
    }

    SerialRS485* getPort(int index)
    {
        if (index < 0 || index >= _ports.size())
            return nullptr;
        return _ports[index];
    }

    size_t portCount() const
    {
        return _ports.size();
    }

    void beginAll(unsigned long baudRate = 115200)
    {
        for (auto port : _ports)
        {
            port->begin(baudRate);
        }
    }
};