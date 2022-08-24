

class LowPassFilter
{
    public:
        LowPassFilter() { }

        LowPassFilter(float a)
        {
            alpha = a;
        }

        void setalpha(float a)
        {
            alpha = a;
        }

        void setInitialValue(float data)
        {
            currentValue = data;
        }

        float addNewData(float data)
        {
            currentValue = (alpha * currentValue) + ((1.0 - alpha) * data);
            return currentValue;
        }

        float getCurrentValue()
        {
            return currentValue;
        }

    private:
        float currentValue;
        float alpha = 0.0;   //Start with no filtering
};