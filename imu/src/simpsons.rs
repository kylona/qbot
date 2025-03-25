
pub struct SimpsonsIntegral {
    partial_sum : f32,
    pub result : f32,
    delta_t : f32,
    buffer : [f32; 2],
}
impl SimpsonsIntegral {
    pub fn default() -> Self {
        Self::new(1.0/500.0, 0.0)
    }
    pub fn new(delta_t : f32, init_sum : f32) -> Self {
        let buffer = [init_sum; 2];
        Self {
            partial_sum: init_sum,
            result: init_sum,
            delta_t: delta_t,
            buffer: buffer,
        }
    }
    pub fn update(&mut self, value : f32) -> f32 {
        let update_sum = self.delta_t/3.0 * (self.buffer[0] + 4.0*self.buffer[1] + value);
        self.result = self.partial_sum + update_sum;
        self.partial_sum += self.delta_t * (self.buffer[0] + self.buffer[1])/2.0;
        self.buffer[0] = self.buffer[1];
        self.buffer[1] = value;
        self.result
    }
    pub fn get_result(&self) -> f32 {
        self.result
    }
}
