#[macro_export]
macro_rules! matrix_error {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident {
            $(
                $variant:ident $({ $($field:ident: $fty:ty),* })? => $msg:literal
            ),*
            $(,)?
        }
        $(
            from {
                $($from_variant:ident($from_ty:ty)),*
                $(,)?
            }
        )?
    ) =>{
        $(#[$meta])*
        $vis enum $name {
            $($variant $({ $($field: $fty),* })?),*
            $($(, $from_variant($from_ty))*)?
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                match self {
                    $(
                        Self::$variant $({ $($field),* })? => write!(f, $msg $(, $($field = $field),*)?),
                    )*
                    $($(
                        Self::$from_variant(inner) => write!(f, "{}", inner),
                    )*)?
                }
            }
        }

        impl std::error::Error for $name {}

        $($(
            impl From<$from_ty> for $name {
                fn from(e: $from_ty) -> Self {
                    $name::$from_variant(e)
                }
            }
        )*)?
    };
}
