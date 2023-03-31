/**
 * A 4x4 transformation matrix.
 * @author Gariam
 */
class MatrixTRS{
    /**
     * @param {(Vec3 | Mat4 | MatrixTRS | number[])} origin - origin vector
     * @param {Vec3=} angles - angles vector in radians
     * @param {Vec3=} scale - scale vector
     */
    constructor(origin = new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]), angles = new Vec3(0), scale = new Vec3(1)){
        if (Array.isArray(array) || array instanceof Float64Array || array instanceof Float32Array){
            /**
             * The matrix as flat array of length 16.
             * @type Float64Array
             */
            this.m = new Float64Array(origin);
        } else if (origin instanceof Mat4 || origin instanceof MatrixTRS){
            this.m = new Float64Array(origin.m);
        } else {
            this.m = new Float64Array(16);
            const rotMatrix = MatrixTRS._anglesToMatrix(angles); //convert angles to rotation matrix

            //1st column (scaled right vector)
            this.m[0] = rotMatrix[0] * scale.x;
            this.m[1] = rotMatrix[1] * scale.x;
            this.m[2] = rotMatrix[2] * scale.x;
            this.m[3] = 0;

            //2nd column (scaled up vector)
            this.m[4] = rotMatrix[4] * scale.y;
            this.m[5] = rotMatrix[5] * scale.y;
            this.m[6] = rotMatrix[6] * scale.y;
            this.m[7] = 0;

            //3rd column (scaled forward vector)
            this.m[8] = rotMatrix[8] * scale.z;
            this.m[9] = rotMatrix[9] * scale.z;
            this.m[10] = rotMatrix[10] * scale.z;
            this.m[11] = 0;

            //4th column (translation)
            this.m[12] = origin.x;
            this.m[13] = origin.y;
            this.m[14] = origin.z;
            this.m[15] = 1;
        }
    }

    /**
     * Returns a 4x4 identity matrix as a flat array.
     * @returns {Float64Array} 4x4 identity matrix
     */
    static identityMatrix(){
        return new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);
    }

    /**
     * Checks if the provided matrix is a valid transformation matrix.
     * @param {(MatrixTRS | Mat4 | number[])} matrix
     * @returns {boolean}
     */
    static isMatrixTRS(matrix){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
        if (!(Array.isArray(array) || array instanceof Float64Array || array instanceof Float32Array)) return false; //check if it's an array
        matrix = new Float64Array(matrix);
        if (matrix.length != 16) return false; //check if it's a 4x4 matrix
        if (matrix[3] != 0 || matrix[7] != 0 || matrix[11] != 0 || matrix[15] != 1) return false; //check if the last row is [0, 0, 0, 1]

        //If the determinant of the submatrix from 00 to 22 is negative or 0 then the matrix is not valid
        const determinant =
            matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
            matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
            matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
        if (determinant <= 0 || isNaN(determinant) || determinant == undefined) return false;

        const transposed = new Float32Array([matrix[0], matrix[4], matrix[8], matrix[1], matrix[5], matrix[9], matrix[2], matrix[6], matrix[10]]);

        const adjugate = new Float32Array([
            matrix[5] * matrix[10] - matrix[6] * matrix[9],
            matrix[2] * matrix[9] - matrix[1] * matrix[10],
            matrix[1] * matrix[6] - matrix[2] * matrix[5],
            matrix[6] * matrix[8] - matrix[4] * matrix[10],
            matrix[0] * matrix[10] - matrix[2] * matrix[8],
            matrix[2] * matrix[4] - matrix[0] * matrix[6],
            matrix[4] * matrix[9] - matrix[5] * matrix[8],
            matrix[1] * matrix[8] - matrix[0] * matrix[9],
            matrix[0] * matrix[5] - matrix[1] * matrix[4]]);
        
        //if the inverse of the submatrix from 00 to 22 is different from the same submatrix transposed then it's not valid
        for (let i = 0; i < adjugate.length; i++){
            const inverse = adjugate[i] / determinant;
            if (inverse != transposed[i]) return false;
        }
        return true;
    }

    /**
     * Multiplies a vector with a matrix and returns the resulting vector.
     * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
     * @param {(Vec3 | Vec2)} vector
     * @returns {Vec3} 3-dimensional vector
     */
    static vectorMultiply(matrix, vector){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = new Float64Array(matrix.m);
        else matrix = new Float64Array(matrix);
        if (vector instanceof Vec2) vector = new Vec3(vector.x, vector.y, 1);
        
        const x = vector.x * matrix[0] + vector.y * matrix[4] + vector.z * matrix[8] + matrix[12];
        const y = vector.x * matrix[1] + vector.y * matrix[5] + vector.z * matrix[9] + matrix[13];
        const z = vector.x * matrix[2] + vector.y * matrix[6] + vector.z * matrix[10] + matrix[14];
    
        return new Vec3(x, y, z);
    }

    /**
     * Multiplies two transformation matrices together and returns the result.
     * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
     * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
     * @returns {MatrixTRS} 4x4 transformation matrix
     */
    static matrixMultiply(matrixA, matrixB){
        if (matrixA instanceof MatrixTRS || matrixA instanceof Mat4) matrixA = new Float64Array(matrixA.m);
        else matrixA = new Float64Array(matrixA);
        if (matrixB instanceof MatrixTRS || matrixB instanceof Mat4) matrixB = new Float64Array(matrixB.m);
        else matrixB = new Float64Array(matrixB);

        const row0 = new Float64Array([matrixB[0], matrixB[1], matrixB[2]]);
        const row1 = new Float64Array([matrixB[4], matrixB[5], matrixB[6]]);
        const row2 = new Float64Array([matrixB[8], matrixB[9], matrixB[10]]);

        const column0 = matrixArrayMultiply(matrixA, row0);
        const column1 = matrixArrayMultiply(matrixA, row1);
        const column2 = matrixArrayMultiply(matrixA, row2);
        const column3 = new Float64Array([matrixA[12] + matrixB[12], matrixA[13] + matrixB[13], matrixA[14] + matrixB[14]]);

        function matrixArrayMultiply(matrix, array) {
            const x = array[0] * matrix[0] + array[1] * matrix[4] + array[2] * matrix[8];
            const y = array[0] * matrix[1] + array[1] * matrix[5] + array[2] * matrix[9];
            const z = array[0] * matrix[2] + array[1] * matrix[6] + array[2] * matrix[10];
            return new Float64Array([x, y, z]);
        }
    
        return new MatrixTRS(new Float64Array([
            column0[0], column0[1], column0[2], 0,
            column1[0], column1[1], column1[2], 0,
            column2[0], column2[1], column2[2], 0,
            column3[0], column3[1], column3[2], 1]
        ));
    }

    /**
     * Spherical linear interpolation between the two matrices.
     * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
     * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
     * @param {number} value - interpolation factor
     * @param {number=} valueRot - rotation interpolation factor
     * @param {number=} valueScl - scale interpolation factor
     * @returns {MatrixTRS}
     */
    static slerp(matrixA, matrixB, value, valueRot, valueScl){
        if (matrixA instanceof MatrixTRS || matrixA instanceof Mat4) matrixA = new Float64Array(matrixA.m);
        else matrixA = new Float64Array(matrixA);
        if (matrixB instanceof MatrixTRS || matrixB instanceof Mat4) matrixB = new Float64Array(matrixB.m);
        else matrixB = new Float64Array(matrixB);
        if (valueRot == undefined) valueRot = value;
        if (valueScl == undefined) valueScl = value;
        if (value == 0 && valueRot == 0 && valueScl == 0) return new MatrixTRS(matrixA);
        else if (value == 1 && valueRot == 1 && valueScl == 1) return new MatrixTRS(matrixB);
        let matrix;

        function mixVec3(vectorA, vectorB, value){
            vectorA.x = vectorA.x * (1 - value) + vectorB.x * value;
            vectorA.y = vectorA.y * (1 - value) + vectorB.y * value;
            vectorA.z = vectorA.z * (1 - value) + vectorB.z * value;
            return vectorA;
        }
        
        //interpolate origin
        const origin = mixVec3(new Vec3(matrixA[12], matrixA[13], matrixA[14]), new Vec3(matrixB[12], matrixB[13], matrixB[14]), value);
        
        //interpolate angles
        const q1 = MatrixTRS._matrixToQuaternion(matrixA);
        const q2 = MatrixTRS._matrixToQuaternion(matrixB);
        const quaternion = MatrixTRS._slerpQuaternion(q1, q2, valueRot);
        matrix = MatrixTRS._quaternionToMatrix(quaternion);

        //interpolate scale
        const scaleA = MatrixTRS._matrixToScale(matrixA);
        const scaleB = MatrixTRS._matrixToScale(matrixB);
        const scale = mixVec3(scaleA, scaleB, valueScl);

        matrix[0] *= scale.x;
        matrix[1] *= scale.x;
        matrix[2] *= scale.x;
        matrix[4] *= scale.y;
        matrix[5] *= scale.y;
        matrix[6] *= scale.y;
        matrix[8] *= scale.z;
        matrix[9] *= scale.z;
        matrix[10] *= scale.z;
        matrix[12] = origin.x;
        matrix[13] = origin.y;
        matrix[14] = origin.z;

        return new MatrixTRS(matrix);
    }

    /**
     * Checks if it's still a valid transformation matrix.
     * @returns {boolean}
     */
    isValid(){
        return MatrixTRS.isMatrixTRS(this.m);
    }

    /**
     * Returns an identical transformation matrix.
     * @returns {MatrixTRS}
     */
    copy(){
        return new MatrixTRS(this.m);
    }

    /**
     * Returns the matrix as a string.
     * @returns {string}
     */
    toString(){
		return this.m[0]+' '+this.m[1]+' '+this.m[2]+' '+this.m[3]+' '+
				this.m[4]+' '+this.m[5]+' '+this.m[6]+' '+this.m[7]+' '+
				this.m[8]+' '+this.m[9]+' '+this.m[10]+' '+this.m[11]+' '+
				this.m[12]+' '+this.m[13]+' '+this.m[14]+' '+this.m[15];
	}

    /**
     * Extracts a transformation matrix with only the specified components and returns it.
     * 
     * Doesn't change the original matrix.
     * @param {boolean} origin
     * @param {boolean} angles
     * @param {boolean} scale
     * @returns {MatrixTRS}
     */
    selectTransforms(origin, angles, scale){
        let newMatrix = new Float64Array(this.m), newScale;

        if (!origin){
            newMatrix[12] = 0;
            newMatrix[13] = 0;
            newMatrix[14] = 0;
        }
        if (!angles){
            if (scale) newScale = this.getScale();
            else newScale = new Vec3(1);
            newMatrix[0] = newScale.x;
            newMatrix[1] = 0;
            newMatrix[2] = 0;
            newMatrix[4] = 0;
            newMatrix[5] = newScale.y;
            newMatrix[6] = 0;
            newMatrix[8] = 0;
            newMatrix[9] = 0;
            newMatrix[10] = newScale.z;
        } else if (!scale) {
            const currentScale = this.getScale();
            newMatrix[0] /= currentScale.x;
            newMatrix[1] /= currentScale.x;
            newMatrix[2] /= currentScale.x;
            newMatrix[4] /= currentScale.y;
            newMatrix[5] /= currentScale.y;
            newMatrix[6] /= currentScale.y;
            newMatrix[8] /= currentScale.z;
            newMatrix[9] /= currentScale.z;
            newMatrix[10] /= currentScale.z;
        }

        return new MatrixTRS(newMatrix);
    }

    /**
     * Sets new transforms for the matrix.
     * @param {(Vec3 | MatrixTRS | Mat4 | number[])} origin
     * @param {Vec3=} angles
     * @param {Vec3=} scale
     */
    setTransforms(origin, angles = new Vec3(0), scale = new Vec3(1)){
        if (Array.isArray(array) || array instanceof Float64Array || array instanceof Float32Array) {
            this.m = new Float64Array(origin);
        } else if (origin instanceof Mat4 || origin instanceof MatrixTRS){
            this.m = new Float64Array(origin.m);
        } else {
            const rotMatrix = MatrixTRS._anglesToMatrix(angles); //convert angles to rotation matrix

            //1st column (scaled right vector)
            this.m[0] = rotMatrix[0] * scale.x;
            this.m[1] = rotMatrix[1] * scale.x;
            this.m[2] = rotMatrix[2] * scale.x;

            //2nd column (scaled up vector)
            this.m[4] = rotMatrix[4] * scale.y;
            this.m[5] = rotMatrix[5] * scale.y;
            this.m[6] = rotMatrix[6] * scale.y;

            //3rd column (scaled forward vector)
            this.m[8] = rotMatrix[8] * scale.z;
            this.m[9] = rotMatrix[9] * scale.z;
            this.m[10] = rotMatrix[10] * scale.z;

            //4th column (translation)
            this.m[12] = origin.x;
            this.m[13] = origin.y;
            this.m[14] = origin.z;
        }
    }

    /**
     * Returns the origin vector
     * @returns {Vec3} origin vector
     */
    getOrigin(){
        return new Vec3(this.m[12], this.m[13], this.m[14]);
    }

    /**
     * Return the angles vector in radians.
     * @returns {Vec3} angles vector in radians
     */
    getAngles(){
        return MatrixTRS._matrixToAngles(this.m);
    }

    /**
     * Returns the scale vector.
     * @returns {Vec3} scale vector
     */
    getScale(){
        return MatrixTRS._matrixToScale(this.m);
    }

    /**
     * Returns the right view vector normalized.
     * @returns {Vec3} normalized right vector
     */
    getRight(){
        return new Vec3(this.m[0], this.m[1], this.m[2]).normalize();
    }

    /**
     * Returns the up view vector normalized.
     * @returns {Vec3} normalized up vector
     */
    getUp(){
        return new Vec3(this.m[4], this.m[5], this.m[6]).normalize();
    }

    /**
     * Returns the forward view vector normalized.
     * @returns {Vec3} normalized forward vector
     */
    getForward(){
        return new Vec3(this.m[8], this.m[9], this.m[10]).normalize();
    }

    /**
     * Returns the longest eigen vector, null if there are none.
     * 
     * Not translated nor normalized.
     * @returns {(Vec3 | null)} eigen vector
     */
    getEigenVector(){
        const a = this.m[0], b = this.m[1], c = this.m[2];
        const d = this.m[4], e = this.m[5], f = this.m[6];
        const g = this.m[8], h = this.m[9], i = this.m[10];

        //Compute determinant and trace of the matrix
        const det = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;
        const trace = a + e + i;

        //Compute eigenvalues using the quadratic formula
        const discriminant = trace * trace - 4 * det;
        if (discriminant < 0) return null;
        const lambda1 = (trace + Math.sqrt(discriminant)) / 2;
        const lambda2 = (trace - Math.sqrt(discriminant)) / 2;

        //Find largest eigenvalue and corresponding eigenvector
        const lambda = (lambda1 > lambda2) ? lambda1 : lambda2;
        let vector = new Vec3(g - i * lambda, h, i - lambda);
        if (Math.abs(vector.x) > Math.abs(vector.y)){
            const temp = vector.x;
            vector.x = vector.y;
            vector.y = temp;
        }
        if (Math.abs(vector.y) > Math.abs(vector.z)){
            const temp = vector.y;
            vector.y = vector.z;
            vector.z = temp;
        }

        return vector;
    }

    /**
     * Changes the origin component of the matrix.
     * @param {Vec3} origin - origin vector
     */
    setOrigin(origin){
        this.m[12] = origin.x;
        this.m[13] = origin.y;
        this.m[14] = origin.z;
    }

    /**
     * Changes the scale component of the matrix.
     * @param {Vec3} scale - scale vector
     */
    setScale(scale){
        const right = this.getRight().multiply(scale.x);
        const up = this.getUp().multiply(scale.y);
        const forward = this.getForward().multiply(scale.z);

        this.m[0] = right.x;
        this.m[1] = right.y;
        this.m[2] = right.z;
        this.m[4] = up.x;
        this.m[5] = up.y;
        this.m[6] = up.z;
        this.m[8] = forward.x;
        this.m[9] = forward.y;
        this.m[10] = forward.z;
    }

    /**
     * Changes the angles component of the matrix.
     * @param {Vec3} angles - angles vector in radians
     */
    setAngles(angles){
        const magnitude = this.getScale();
        const rotMatrix = MatrixTRS._anglesToMatrix(angles);

        //right vector
        this.m[0] = rotMatrix[0] * magnitude.x;
        this.m[1] = rotMatrix[1] * magnitude.x;
        this.m[2] = rotMatrix[2] * magnitude.x;

        //up vector
        this.m[4] = rotMatrix[4] * magnitude.y;
        this.m[5] = rotMatrix[5] * magnitude.y;
        this.m[6] = rotMatrix[6] * magnitude.y;

        //forward vector
        this.m[8] = rotMatrix[8] * magnitude.z;
        this.m[9] = rotMatrix[9] * magnitude.z;
        this.m[10] = rotMatrix[10] * magnitude.z;
    }

    /**
     * Changes the view vectors of the matrix.
     * @param {Vec3} right - normalized right vector
     * @param {Vec3} up - normalized up vector
     * @param {Vec3} forward - normalized forward vector
     */
    setViewVectors(right, up, forward){
        const scale = this.getScale();
        right = right.multiply(scale.x);
        up = up.multiply(scale.y);
        forward = forward.multiply(scale.z);

        this.m[0] = right.x;
        this.m[1] = right.y;
        this.m[2] = right.z;
        this.m[4] = up.x;
        this.m[5] = up.y;
        this.m[6] = up.z;
        this.m[8] = forward.x;
        this.m[9] = forward.y;
        this.m[10] = forward.z;
    }

    /**
     * Applies a translation and returns the resulting matrix.
     * 
     * Doesn't change the original matrix.
     * @param {Vec3} translation - translation vector
     * @returns {MatrixTRS} translated matrix
     */
    translate(translation) {
        let newMatrix = new Float64Array(this.m);
        newMatrix[12] += translation.x;
        newMatrix[13] += translation.y;
        newMatrix[14] += translation.z;
        return new MatrixTRS(newMatrix);
    }

    /**
     * Applies a rotation and returns the resulting matrix.
     * 
     * Doesn't change the original matrix.
     * @param {Vec3} angles - angles vector in radians
     * @returns {MatrixTRS} rotated matrix
     */
    rotate(angles) {
        const rotMatrix = MatrixTRS._anglesToMatrix(angles); //convert angles to rotation matrix
        return MatrixTRS.matrixMultiply(this.m, rotMatrix); //multiplies to rotation matrix with the current transformation matrix
    }

    /**
     * Applies scaling and returns the resulting matrix.
     * 
     * Doesn't change the original matrix.
     * @param {Vec3} scale - scale vector
     * @returns {MatrixTRS} scaled matrix
     */
    scale(scale) {
        let newMatrix = new Float64Array(this.m);
        newMatrix[0] *= scale.x;
        newMatrix[1] *= scale.x;
        newMatrix[2] *= scale.x;
        newMatrix[4] *= scale.y;
        newMatrix[5] *= scale.y;
        newMatrix[6] *= scale.y;
        newMatrix[8] *= scale.z;
        newMatrix[9] *= scale.z;
        newMatrix[10] *= scale.z;
        return new MatrixTRS(newMatrix);
    }

    /**
     * Multiplies the matrix and returns the result.
     * 
     * Doesn't change the original matrix.
     * @param {(MatrixTRS | Mat4 | Vec3 | Vec2 | number[])} multiplier - 4x4 transformation matrix or a vector
     * @returns {(MatrixTRS | Vec3)} 4x4 transformation matrix or a vector
     */
    multiply(multiplier) {
        if (multiplier instanceof Mat4 || multiplier instanceof MatrixTRS || Array.isArray(array) || array instanceof Float64Array || array instanceof Float32Array){
            return MatrixTRS.matrixMultiply(this.m, multiplier);
        } else if (multiplier instanceof Vec2 || multiplier instanceof Vec3){
            return MatrixTRS.vectorMultiply(this.m, multiplier);
        }
    }

    /**
     * Returns the inverse of the matrix.
     * 
     * Doesn't change the original matrix.
     * @returns {MatrixTRS} inverted 4x4 transformation matrix
     */
    inverse(){
        let inverted = MatrixTRS.identityMatrix();
        let right = new Vec3(this.m[0], this.m[1], this.m[2]);
        let up = new Vec3(this.m[4], this.m[5], this.m[6]);
        let forward = new Vec3(this.m[8], this.m[9], this.m[10]);

        //Inverse scale
        right = right.divide(right.lengthSqr());
        up = up.divide(up.lengthSqr());
        forward = forward.divide(forward.lengthSqr());

        //Inverse rotation
        inverted[0] = right.x;
        inverted[1] = up.x;
        inverted[2] = forward.x;
        inverted[4] = right.y;
        inverted[5] = up.y;
        inverted[6] = forward.y;
        inverted[8] = right.z;
        inverted[9] = up.z;
        inverted[10] = forward.z;

        //Inverse translation
        inverted[12] = -this.m[12];
        inverted[13] = -this.m[13];
        inverted[14] = -this.m[14];

        return new MatrixTRS(inverted);
    }

    static _anglesToMatrix(angles){
        const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
        const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
        const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

        const sinAlpha_sinBeta = sinAlpha * sinBeta;
        const cosAlpha_sinBeta = cosAlpha * sinBeta;

        const m11 = cosGamma * cosBeta;
        const m12 = sinGamma * cosBeta;
        const m13 = -sinBeta;
        const m21 = cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha;
        const m22 = sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha;
        const m23 = cosBeta * sinAlpha;
        const m31 = cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha;
        const m32 = sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha;
        const m33 = cosBeta * cosAlpha;

        return new Float64Array([m11, m12, m13, 0, m21, m22, m23, 0, m31, m32, m33, 0, 0, 0, 0, 1]);
    }

    static _matrixToAngles(matrix){
        const right = new Vec3(matrix[0], matrix[1], matrix[2]).normalize();
        const up = new Vec3(matrix[4], matrix[5], matrix[6]).normalize();
        const forward = new Vec3(matrix[8], matrix[9], matrix[10]).normalize();
        let angles = new Vec3();
        
        if (1.0 - Math.abs(right.z) > 1e-6){
            angles.x = Math.atan2(up.z, forward.z);
            angles.y = -Math.asin(right.z);
            angles.z = Math.atan2(right.y, right.x);
        } else {
            if (right.z > 0){
                angles.x = 1.5707963268;
                angles.y = Math.atan2(right.x, up.x);
                angles.z = 0;
            } else {
                angles.x = -1.5707963268;
                angles.y = -Math.atan2(right.x, up.x);
                angles.z = 0;
            }
        }
        return angles;
    }

    static _matrixToScale(matrix){
        const x = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const y = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const z = new Vec3(matrix[8], matrix[9], matrix[10]).length();
        return new Vec3(x, y, z);
    }

    static _matrixToQuaternion(matrix) {
        const trace = matrix[0] + matrix[5] + matrix[10];
        let w, x, y, z;

        if (trace > 0) {
            const s = 0.5 / Math.sqrt(trace + 1);
            w = 0.25 / s;
            x = (matrix[9] - matrix[6]) * s;
            y = (matrix[2] - matrix[8]) * s;
            z = (matrix[4] - matrix[1]) * s;
        } else if (matrix[0] > matrix[5] && matrix[0] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[0] - matrix[5] - matrix[10]);
            w = (matrix[9] - matrix[6]) / s;
            x = 0.25 * s;
            y = (matrix[1] + matrix[4]) / s;
            z = (matrix[2] + matrix[8]) / s;
        } else if (matrix[5] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[5] - matrix[0] - matrix[10]);
            w = (matrix[2] - matrix[8]) / s;
            x = (matrix[1] + matrix[4]) / s;
            y = 0.25 * s;
            z = (matrix[6] + matrix[9]) / s;
        } else {
            const s = 2 * Math.sqrt(1 + matrix[10] - matrix[0] - matrix[5]);
            w = (matrix[4] - matrix[1]) / s;
            x = (matrix[2] + matrix[8]) / s;
            y = (matrix[6] + matrix[9]) / s;
            z = 0.25 * s;
        }

        const length = Math.sqrt(w * w + x * x + y * y + z * z);
        return new Float64Array([w / length, x / length, y / length, z / length]);
    }

    static _quaternionToMatrix(q) {
        const w = q[0], x = q[1], y = q[2], z = q[3];
        const xx = x * x, xy = x * y, xz = x * z, xw = x * w;
        const yy = y * y, yz = y * z, yw = y * w;
        const zz = z * z, zw = z * w;
        return new Float64Array([
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0,
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0,
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1]);
    }

    static _slerpQuaternion(q1, q2, t) {
        let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        if (dot < 0) {
            q2 = new Float64Array(q2.map(value => -value));
            dot = -dot;
        }

        if (dot > 0.9995) {
            return new Float64Array([q1[0] * (1 - t) + q2[0] * t,
                    q1[1] * (1 - t) + q2[1] * t,
                    q1[2] * (1 - t) + q2[2] * t,
                    q1[3] * (1 - t) + q2[3] * t]);
        }

        const theta = Math.acos(dot);
        const sinTheta = Math.sin(theta);
        const w1 = Math.sin((1 - t) * theta) / sinTheta;
        const w2 = Math.sin(t * theta) / sinTheta;

        return new Float64Array([
                q1[0] * w1 + q2[0] * w2,
                q1[1] * w1 + q2[1] * w2,
                q1[2] * w1 + q2[2] * w2,
                q1[3] * w1 + q2[3] * w2]);
    }
}