pub mod matrix_handler {
    use crate::{
        handlers::profiles::ModuleProfile, 
        matrix::core::AtomicMatrix
    };
    use memmap2::MmapMut;

    pub struct MatrixHandler {
        pub matrix: &'static mut AtomicMatrix,
        pub mmap: MmapMut,
        // pub profile: ModuleProfile,
        // pub processed_messages: u64,
        // pub bytes_consumed: u64,
        // pub published_messages: u64,
        // pub bytes_allocated: u64,
        // pub current_usage: u64,
        // pub claimed_buffers: Vec<u64>,
    }

    pub struct AdminHandler<T> {
        pub matrix: &'static mut AtomicMatrix,
        pub mmap: MmapMut,
        pub message_protocol: T,
        pub registered_modules: [ModuleProfile; 64],
    }

    impl HandlerFunctions for MatrixHandler {}

    impl<T> HandlerFunctions for AdminHandler<T> {}

    trait HandlerFunctions {
        fn submit() {}

        fn respond() {}

        fn request() {}

        fn inquire() {}

        fn query_inbox() {}

        fn notify() {}

        fn read() {}
    }
}

pub mod profiles {
    pub struct ModuleProfile {}
    pub struct AdminProfile {}
    pub struct PublicProfile {}
    pub struct RegistrationEndpoint{}

    trait ProfileFunctions {}
}

pub mod public {
    use super::*;
    use std::marker::PhantomData;
    use crate::matrix::core::AtomicMatrix;

    pub struct PublicHandler<T> {
        pub matrix: Option<&'static mut AtomicMatrix>,
        pub _public_protocol: PhantomData<T>,
        pub profile: profiles::PublicProfile
    }

    pub struct PublicRegistrationService {
        pub matrix_key: String,
        pub endpoint: profiles::RegistrationEndpoint
    }
}