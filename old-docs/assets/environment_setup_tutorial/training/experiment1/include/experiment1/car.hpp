#ifndef MYPROJECT_MYCLASS_HPP
#define MYPROJECT_MYCLASS_HPP

/**
 * @brief 
 * Class for car
 */
class Car
{
private:
    int speed;
    int weight;
public:
    /// @brief 
    /// @param speed 
    /// @param weight 
    Car(int speed, int weight);
    ~Car();

    void use_horn();
};

#endif 