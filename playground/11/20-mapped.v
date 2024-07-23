// Benchmark "adder" written by ABC on Wed Jul 17 17:44:55 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n343, new_n344, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n06x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\a[2] ), .o1(new_n105));
  aoi022aa1n09x5               g010(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n106));
  aoib12aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(\b[1] ), .out0(new_n107));
  oa0022aa1n06x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  oaoi13aa1n04x5               g013(.a(new_n99), .b(new_n108), .c(new_n107), .d(new_n104), .o1(new_n109));
  nor002aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand42aa1d28x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n03x4               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor002aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  inv000aa1d42x5               g020(.a(\b[6] ), .o1(new_n116));
  nanb02aa1n02x5               g021(.a(\a[7] ), .b(new_n116), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanp03aa1n02x5               g023(.a(new_n115), .b(new_n117), .c(new_n118), .o1(new_n119));
  xorc02aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .out0(new_n120));
  nano22aa1n03x7               g025(.a(new_n119), .b(new_n120), .c(new_n112), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(new_n109), .b(new_n121), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n113), .o1(new_n123));
  aoi112aa1n06x5               g028(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n113), .b(new_n114), .out0(new_n126));
  nanp02aa1n02x5               g031(.a(new_n117), .b(new_n118), .o1(new_n127));
  oai022aa1n02x5               g032(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n128));
  nano23aa1n02x4               g033(.a(new_n126), .b(new_n127), .c(new_n128), .d(new_n111), .out0(new_n129));
  nano22aa1n03x7               g034(.a(new_n129), .b(new_n123), .c(new_n125), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n97), .b(new_n131), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n98), .b(new_n132), .c(new_n122), .d(new_n130), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand22aa1n03x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  oai012aa1n06x5               g041(.a(new_n136), .b(new_n97), .c(new_n135), .o1(new_n137));
  nona23aa1n02x4               g042(.a(new_n131), .b(new_n136), .c(new_n135), .d(new_n97), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n138), .c(new_n122), .d(new_n130), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand22aa1n09x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n143));
  nor002aa1d32x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  nand02aa1d08x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n143), .b(new_n146), .c(new_n145), .out0(\s[12] ));
  nano23aa1n06x5               g052(.a(new_n135), .b(new_n97), .c(new_n131), .d(new_n136), .out0(new_n148));
  nano23aa1n03x7               g053(.a(new_n141), .b(new_n144), .c(new_n146), .d(new_n142), .out0(new_n149));
  nand02aa1n02x5               g054(.a(new_n149), .b(new_n148), .o1(new_n150));
  nona23aa1d18x5               g055(.a(new_n146), .b(new_n142), .c(new_n141), .d(new_n144), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n141), .b(new_n146), .o1(new_n152));
  oai112aa1n06x5               g057(.a(new_n152), .b(new_n145), .c(new_n151), .d(new_n137), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n122), .d(new_n130), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g065(.a(new_n111), .o1(new_n161));
  xorc02aa1n12x5               g066(.a(\a[7] ), .b(\b[6] ), .out0(new_n162));
  inv000aa1d42x5               g067(.a(\a[5] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[4] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n110), .b(new_n163), .c(new_n164), .o1(new_n165));
  nona23aa1n03x5               g070(.a(new_n162), .b(new_n115), .c(new_n165), .d(new_n161), .out0(new_n166));
  nona22aa1n03x5               g071(.a(new_n166), .b(new_n124), .c(new_n113), .out0(new_n167));
  norp02aa1n02x5               g072(.a(new_n151), .b(new_n138), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n167), .c(new_n109), .d(new_n121), .o1(new_n169));
  nor022aa1n16x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n171), .b(new_n158), .c(new_n157), .d(new_n170), .out0(new_n172));
  oai012aa1n02x5               g077(.a(new_n171), .b(new_n170), .c(new_n157), .o1(new_n173));
  aoai13aa1n02x7               g078(.a(new_n173), .b(new_n172), .c(new_n169), .d(new_n154), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n06x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand42aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  nor002aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n176), .c(new_n174), .d(new_n178), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n181), .b(new_n176), .c(new_n174), .d(new_n177), .o1(new_n183));
  norb02aa1n02x7               g088(.a(new_n183), .b(new_n182), .out0(\s[16] ));
  nona23aa1n03x5               g089(.a(new_n180), .b(new_n177), .c(new_n176), .d(new_n179), .out0(new_n185));
  nor043aa1n02x5               g090(.a(new_n150), .b(new_n172), .c(new_n185), .o1(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n167), .c(new_n109), .d(new_n121), .o1(new_n187));
  nor042aa1n02x5               g092(.a(new_n185), .b(new_n172), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n189));
  oai022aa1n02x5               g094(.a(new_n185), .b(new_n173), .c(\b[15] ), .d(\a[16] ), .o1(new_n190));
  aoi112aa1n09x5               g095(.a(new_n190), .b(new_n189), .c(new_n153), .d(new_n188), .o1(new_n191));
  nand02aa1d06x5               g096(.a(new_n191), .b(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nand42aa1n03x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nanb02aa1n06x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n99), .o1(new_n200));
  xorc02aa1n02x5               g105(.a(\a[3] ), .b(\b[2] ), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .o1(new_n202));
  aob012aa1n02x5               g107(.a(new_n202), .b(\b[1] ), .c(\a[2] ), .out0(new_n203));
  oaib12aa1n02x5               g108(.a(new_n203), .b(\b[1] ), .c(new_n105), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n108), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n200), .b(new_n205), .c(new_n204), .d(new_n201), .o1(new_n206));
  nona23aa1n02x4               g111(.a(new_n120), .b(new_n112), .c(new_n126), .d(new_n127), .out0(new_n207));
  nand02aa1n02x5               g112(.a(new_n188), .b(new_n168), .o1(new_n208));
  oaoi13aa1n09x5               g113(.a(new_n208), .b(new_n130), .c(new_n206), .d(new_n207), .o1(new_n209));
  nanp02aa1n03x5               g114(.a(new_n153), .b(new_n188), .o1(new_n210));
  nona22aa1n03x5               g115(.a(new_n210), .b(new_n190), .c(new_n189), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n209), .o1(new_n213));
  xobna2aa1n03x5               g118(.a(new_n199), .b(new_n213), .c(new_n196), .out0(\s[18] ));
  nano22aa1d15x5               g119(.a(new_n199), .b(new_n196), .c(new_n212), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n04x5               g121(.a(new_n198), .b(new_n197), .c(new_n194), .d(new_n195), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n191), .d(new_n187), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand02aa1d04x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nanb02aa1d24x5               g127(.a(new_n221), .b(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[19] ), .o1(new_n225));
  nanb02aa1n03x5               g130(.a(\a[20] ), .b(new_n225), .out0(new_n226));
  nanp02aa1n02x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nanp02aa1n06x5               g132(.a(new_n226), .b(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi112aa1n02x7               g134(.a(new_n221), .b(new_n229), .c(new_n218), .d(new_n224), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n221), .o1(new_n231));
  tech160nm_finand02aa1n05x5   g136(.a(new_n218), .b(new_n224), .o1(new_n232));
  aoi012aa1n03x5               g137(.a(new_n228), .b(new_n232), .c(new_n231), .o1(new_n233));
  norp02aa1n03x5               g138(.a(new_n233), .b(new_n230), .o1(\s[20] ));
  nona22aa1n02x4               g139(.a(new_n215), .b(new_n223), .c(new_n228), .out0(new_n235));
  nanp02aa1n02x5               g140(.a(new_n221), .b(new_n227), .o1(new_n236));
  nor003aa1n03x5               g141(.a(new_n217), .b(new_n223), .c(new_n228), .o1(new_n237));
  nano22aa1n03x7               g142(.a(new_n237), .b(new_n226), .c(new_n236), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n235), .c(new_n191), .d(new_n187), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  tech160nm_fixnrc02aa1n05x5   g146(.a(\b[20] ), .b(\a[21] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi112aa1n02x7               g150(.a(new_n241), .b(new_n245), .c(new_n239), .d(new_n243), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n241), .o1(new_n247));
  tech160nm_finand02aa1n05x5   g152(.a(new_n239), .b(new_n243), .o1(new_n248));
  aoi012aa1n03x5               g153(.a(new_n244), .b(new_n248), .c(new_n247), .o1(new_n249));
  norp02aa1n03x5               g154(.a(new_n249), .b(new_n246), .o1(\s[22] ));
  norp02aa1n02x5               g155(.a(\b[19] ), .b(\a[20] ), .o1(new_n251));
  nona23aa1n06x5               g156(.a(new_n227), .b(new_n222), .c(new_n221), .d(new_n251), .out0(new_n252));
  nor042aa1n06x5               g157(.a(new_n244), .b(new_n242), .o1(new_n253));
  nanb03aa1n09x5               g158(.a(new_n252), .b(new_n253), .c(new_n215), .out0(new_n254));
  oai112aa1n02x7               g159(.a(new_n236), .b(new_n226), .c(new_n252), .d(new_n217), .o1(new_n255));
  oaoi03aa1n09x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n255), .c(new_n253), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n254), .c(new_n191), .d(new_n187), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[23] ), .b(\b[22] ), .out0(new_n261));
  xorc02aa1n12x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  aoi112aa1n02x7               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n260), .o1(new_n264));
  nand42aa1n02x5               g169(.a(new_n258), .b(new_n261), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n262), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .o1(new_n267));
  norp02aa1n03x5               g172(.a(new_n267), .b(new_n263), .o1(\s[24] ));
  nanp02aa1n02x5               g173(.a(new_n262), .b(new_n261), .o1(new_n269));
  nona23aa1n02x4               g174(.a(new_n253), .b(new_n215), .c(new_n269), .d(new_n252), .out0(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[22] ), .b(\a[23] ), .out0(new_n271));
  norb02aa1n02x5               g176(.a(new_n262), .b(new_n271), .out0(new_n272));
  norp02aa1n02x5               g177(.a(\b[23] ), .b(\a[24] ), .o1(new_n273));
  aoi112aa1n02x5               g178(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n274));
  nanp03aa1n03x5               g179(.a(new_n256), .b(new_n261), .c(new_n262), .o1(new_n275));
  nona22aa1n09x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .out0(new_n276));
  aoi013aa1n02x4               g181(.a(new_n276), .b(new_n255), .c(new_n253), .d(new_n272), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n270), .c(new_n191), .d(new_n187), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  xorc02aa1n12x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  aoi112aa1n02x7               g187(.a(new_n280), .b(new_n282), .c(new_n278), .d(new_n281), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n280), .o1(new_n284));
  nanp02aa1n06x5               g189(.a(new_n278), .b(new_n281), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aoi012aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n283), .o1(\s[26] ));
  and002aa1n02x5               g193(.a(new_n282), .b(new_n281), .o(new_n289));
  nano22aa1n03x7               g194(.a(new_n254), .b(new_n289), .c(new_n272), .out0(new_n290));
  oai012aa1n06x5               g195(.a(new_n290), .b(new_n211), .c(new_n209), .o1(new_n291));
  nano22aa1n03x7               g196(.a(new_n238), .b(new_n253), .c(new_n272), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  oaoi13aa1n09x5               g199(.a(new_n294), .b(new_n289), .c(new_n292), .d(new_n276), .o1(new_n295));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n297), .b(new_n296), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n291), .c(new_n295), .out0(\s[27] ));
  inv020aa1n02x5               g204(.a(new_n296), .o1(new_n300));
  xnrc02aa1n12x5               g205(.a(\b[27] ), .b(\a[28] ), .out0(new_n301));
  nona32aa1n02x4               g206(.a(new_n255), .b(new_n269), .c(new_n244), .d(new_n242), .out0(new_n302));
  inv000aa1n02x5               g207(.a(new_n276), .o1(new_n303));
  inv000aa1n02x5               g208(.a(new_n289), .o1(new_n304));
  aoai13aa1n12x5               g209(.a(new_n293), .b(new_n304), .c(new_n302), .d(new_n303), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n297), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n301), .b(new_n306), .c(new_n300), .o1(new_n307));
  aoi022aa1n03x5               g212(.a(new_n291), .b(new_n295), .c(\b[26] ), .d(\a[27] ), .o1(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n300), .c(new_n301), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[28] ));
  nano22aa1d15x5               g215(.a(new_n301), .b(new_n300), .c(new_n297), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n313));
  xnrc02aa1n12x5               g218(.a(\b[28] ), .b(\a[29] ), .out0(new_n314));
  aoi012aa1n03x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n311), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n291), .c(new_n295), .o1(new_n317));
  nano22aa1n02x4               g222(.a(new_n317), .b(new_n313), .c(new_n314), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n315), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n202), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g225(.a(new_n298), .b(new_n314), .c(new_n301), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n313), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .out0(new_n324));
  aoi012aa1n03x5               g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n321), .o1(new_n326));
  aoi012aa1n02x5               g231(.a(new_n326), .b(new_n291), .c(new_n295), .o1(new_n327));
  nano22aa1n03x5               g232(.a(new_n327), .b(new_n323), .c(new_n324), .out0(new_n328));
  nor002aa1n02x5               g233(.a(new_n325), .b(new_n328), .o1(\s[30] ));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  norb03aa1n03x5               g235(.a(new_n311), .b(new_n324), .c(new_n314), .out0(new_n331));
  inv000aa1d42x5               g236(.a(new_n331), .o1(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n291), .c(new_n295), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n334));
  nano22aa1n02x4               g239(.a(new_n333), .b(new_n330), .c(new_n334), .out0(new_n335));
  aoai13aa1n06x5               g240(.a(new_n331), .b(new_n305), .c(new_n192), .d(new_n290), .o1(new_n336));
  tech160nm_fiaoi012aa1n05x5   g241(.a(new_n330), .b(new_n336), .c(new_n334), .o1(new_n337));
  nor002aa1n02x5               g242(.a(new_n337), .b(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g243(.a(new_n107), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g244(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g246(.a(new_n206), .b(\b[4] ), .c(new_n163), .out0(\s[5] ));
  nanp02aa1n02x5               g247(.a(new_n206), .b(new_n120), .o1(new_n343));
  oai112aa1n02x5               g248(.a(new_n343), .b(new_n112), .c(new_n164), .d(new_n163), .o1(new_n344));
  oaoi13aa1n02x5               g249(.a(new_n112), .b(new_n343), .c(new_n163), .d(new_n164), .o1(new_n345));
  norb02aa1n02x5               g250(.a(new_n344), .b(new_n345), .out0(\s[6] ));
  orn002aa1n02x5               g251(.a(\a[6] ), .b(\b[5] ), .o(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n162), .b(new_n344), .c(new_n347), .out0(\s[7] ));
  aoai13aa1n03x5               g253(.a(new_n117), .b(new_n127), .c(new_n344), .d(new_n347), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g255(.a(new_n132), .b(new_n122), .c(new_n130), .out0(\s[9] ));
endmodule


