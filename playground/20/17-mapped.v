// Benchmark "adder" written by ABC on Wed Jul 17 22:20:24 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n335, new_n338, new_n339, new_n340,
    new_n342, new_n343, new_n344, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand22aa1n09x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n06x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[3] ), .o1(new_n102));
  oai022aa1n02x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  oaib12aa1n02x7               g008(.a(new_n103), .b(new_n102), .c(\a[4] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[4] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n102), .b(new_n105), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand02aa1n04x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oai112aa1n06x5               g014(.a(new_n106), .b(new_n107), .c(new_n109), .d(new_n108), .o1(new_n110));
  orn002aa1n03x5               g015(.a(\a[3] ), .b(\b[2] ), .o(new_n111));
  nand02aa1n03x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  oai112aa1n06x5               g017(.a(new_n111), .b(new_n112), .c(new_n102), .d(new_n105), .o1(new_n113));
  oai012aa1n12x5               g018(.a(new_n104), .b(new_n110), .c(new_n113), .o1(new_n114));
  nor002aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanp02aa1n03x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanp02aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n06x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  nor002aa1d32x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1d06x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nor042aa1n06x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nand22aa1n03x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  nona23aa1n03x5               g028(.a(new_n123), .b(new_n121), .c(new_n120), .d(new_n122), .out0(new_n124));
  nor042aa1n06x5               g029(.a(new_n124), .b(new_n119), .o1(new_n125));
  nano22aa1n03x7               g030(.a(new_n122), .b(new_n121), .c(new_n123), .out0(new_n126));
  oaih22aa1n04x5               g031(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n127));
  oai112aa1n02x5               g032(.a(new_n127), .b(new_n116), .c(\b[7] ), .d(\a[8] ), .o1(new_n128));
  aoi012aa1d18x5               g033(.a(new_n120), .b(new_n122), .c(new_n121), .o1(new_n129));
  oaib12aa1n06x5               g034(.a(new_n129), .b(new_n128), .c(new_n126), .out0(new_n130));
  xorc02aa1n12x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n130), .c(new_n114), .d(new_n125), .o1(new_n132));
  xobna2aa1n03x5               g037(.a(new_n99), .b(new_n132), .c(new_n101), .out0(\s[10] ));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  nona22aa1n06x5               g041(.a(new_n132), .b(new_n100), .c(new_n97), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n136), .b(new_n137), .c(new_n98), .out0(\s[11] ));
  nor002aa1d32x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nand22aa1n12x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  aoi013aa1n06x4               g046(.a(new_n134), .b(new_n137), .c(new_n135), .d(new_n98), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n141), .out0(\s[12] ));
  nona23aa1d18x5               g048(.a(new_n141), .b(new_n135), .c(new_n134), .d(new_n139), .out0(new_n144));
  norb03aa1n09x5               g049(.a(new_n131), .b(new_n144), .c(new_n99), .out0(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n130), .c(new_n114), .d(new_n125), .o1(new_n146));
  nand22aa1n09x5               g051(.a(new_n114), .b(new_n125), .o1(new_n147));
  aoi012aa1n02x7               g052(.a(new_n120), .b(\a[6] ), .c(\b[5] ), .o1(new_n148));
  inv000aa1n02x5               g053(.a(new_n129), .o1(new_n149));
  aoi013aa1n06x5               g054(.a(new_n149), .b(new_n126), .c(new_n127), .d(new_n148), .o1(new_n150));
  nona22aa1n02x4               g055(.a(new_n131), .b(new_n144), .c(new_n99), .out0(new_n151));
  nanb03aa1n06x5               g056(.a(new_n139), .b(new_n141), .c(new_n135), .out0(new_n152));
  inv000aa1n02x5               g057(.a(new_n134), .o1(new_n153));
  oai112aa1n03x5               g058(.a(new_n153), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n154));
  tech160nm_fiaoi012aa1n03p5x5 g059(.a(new_n139), .b(new_n134), .c(new_n141), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n152), .o1(new_n156));
  inv020aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n151), .c(new_n147), .d(new_n150), .o1(new_n158));
  nor002aa1d32x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand22aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  nano22aa1n02x5               g066(.a(new_n139), .b(new_n135), .c(new_n141), .out0(new_n162));
  oai012aa1n02x5               g067(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n163));
  oab012aa1n03x5               g068(.a(new_n163), .b(new_n97), .c(new_n100), .out0(new_n164));
  inv040aa1n06x5               g069(.a(new_n155), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n159), .o1(new_n166));
  aoi122aa1n02x5               g071(.a(new_n165), .b(new_n160), .c(new_n166), .d(new_n164), .e(new_n162), .o1(new_n167));
  aboi22aa1n03x5               g072(.a(new_n161), .b(new_n158), .c(new_n167), .d(new_n146), .out0(\s[13] ));
  aoai13aa1n02x5               g073(.a(new_n166), .b(new_n161), .c(new_n146), .d(new_n157), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n04x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nona23aa1n12x5               g077(.a(new_n172), .b(new_n160), .c(new_n159), .d(new_n171), .out0(new_n173));
  tech160nm_fioai012aa1n03p5x5 g078(.a(new_n172), .b(new_n171), .c(new_n159), .o1(new_n174));
  aoai13aa1n04x5               g079(.a(new_n174), .b(new_n173), .c(new_n146), .d(new_n157), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  xnrc02aa1n12x5               g082(.a(\b[14] ), .b(\a[15] ), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  tech160nm_fixnrc02aa1n04x5   g084(.a(\b[15] ), .b(\a[16] ), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n177), .c(new_n175), .d(new_n179), .o1(new_n181));
  aoi112aa1n03x5               g086(.a(new_n177), .b(new_n180), .c(new_n175), .d(new_n179), .o1(new_n182));
  nanb02aa1n03x5               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  inv040aa1n02x5               g088(.a(new_n173), .o1(new_n184));
  nor042aa1n06x5               g089(.a(new_n180), .b(new_n178), .o1(new_n185));
  nano22aa1n03x7               g090(.a(new_n151), .b(new_n184), .c(new_n185), .out0(new_n186));
  aob012aa1n06x5               g091(.a(new_n186), .b(new_n147), .c(new_n150), .out0(new_n187));
  inv000aa1n02x5               g092(.a(new_n174), .o1(new_n188));
  oaoi13aa1n06x5               g093(.a(new_n173), .b(new_n155), .c(new_n154), .d(new_n152), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[16] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[15] ), .o1(new_n191));
  oao003aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n177), .carry(new_n192));
  oaoi13aa1n12x5               g097(.a(new_n192), .b(new_n185), .c(new_n189), .d(new_n188), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n193), .c(new_n187), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[18] ), .o1(new_n196));
  nona32aa1n09x5               g101(.a(new_n145), .b(new_n180), .c(new_n178), .d(new_n173), .out0(new_n197));
  aoi012aa1d24x5               g102(.a(new_n197), .b(new_n147), .c(new_n150), .o1(new_n198));
  inv030aa1n02x5               g103(.a(new_n185), .o1(new_n199));
  aoai13aa1n06x5               g104(.a(new_n184), .b(new_n165), .c(new_n164), .d(new_n162), .o1(new_n200));
  inv000aa1n02x5               g105(.a(new_n192), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n201), .b(new_n199), .c(new_n200), .d(new_n174), .o1(new_n202));
  nor022aa1n16x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  oaoi13aa1n06x5               g108(.a(new_n203), .b(new_n194), .c(new_n202), .d(new_n198), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[17] ), .c(new_n196), .out0(\s[18] ));
  nor002aa1d32x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nand02aa1d20x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nano22aa1n12x5               g112(.a(new_n206), .b(new_n194), .c(new_n207), .out0(new_n208));
  oai012aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n198), .o1(new_n209));
  oa0012aa1n02x5               g114(.a(new_n207), .b(new_n206), .c(new_n203), .o(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor042aa1d18x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1d06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n03x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g121(.a(new_n209), .b(new_n211), .o1(new_n217));
  nor042aa1n09x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  aoai13aa1n03x5               g125(.a(new_n220), .b(new_n212), .c(new_n217), .d(new_n213), .o1(new_n221));
  aoi012aa1n06x5               g126(.a(new_n130), .b(new_n114), .c(new_n125), .o1(new_n222));
  oai012aa1n12x5               g127(.a(new_n193), .b(new_n197), .c(new_n222), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n214), .b(new_n210), .c(new_n223), .d(new_n208), .o1(new_n224));
  nona22aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n212), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n221), .b(new_n225), .o1(\s[20] ));
  nanb03aa1d24x5               g131(.a(new_n220), .b(new_n208), .c(new_n214), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  oai012aa1n02x5               g133(.a(new_n228), .b(new_n202), .c(new_n198), .o1(new_n229));
  nanb03aa1n06x5               g134(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n230));
  oai122aa1n09x5               g135(.a(new_n207), .b(new_n206), .c(new_n203), .d(\b[18] ), .e(\a[19] ), .o1(new_n231));
  aoi012aa1d18x5               g136(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n232));
  oai012aa1n18x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n227), .c(new_n193), .d(new_n187), .o1(new_n235));
  nor002aa1d32x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nand42aa1n08x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  nano22aa1n03x7               g143(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n239));
  tech160nm_fioai012aa1n03p5x5 g144(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  oab012aa1n06x5               g145(.a(new_n240), .b(new_n203), .c(new_n206), .out0(new_n241));
  inv020aa1n03x5               g146(.a(new_n232), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n238), .c(new_n241), .d(new_n239), .o1(new_n243));
  aoi022aa1n02x5               g148(.a(new_n235), .b(new_n238), .c(new_n229), .d(new_n243), .o1(\s[21] ));
  nor002aa1n20x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nand42aa1n08x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n236), .c(new_n235), .d(new_n238), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n238), .b(new_n233), .c(new_n223), .d(new_n228), .o1(new_n249));
  nona22aa1n02x4               g154(.a(new_n249), .b(new_n247), .c(new_n236), .out0(new_n250));
  nanp02aa1n03x5               g155(.a(new_n248), .b(new_n250), .o1(\s[22] ));
  nano23aa1d15x5               g156(.a(new_n236), .b(new_n245), .c(new_n246), .d(new_n237), .out0(new_n252));
  nano32aa1n03x7               g157(.a(new_n220), .b(new_n208), .c(new_n252), .d(new_n214), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  oa0012aa1n03x5               g159(.a(new_n246), .b(new_n245), .c(new_n236), .o(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n233), .c(new_n252), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n254), .c(new_n193), .d(new_n187), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[23] ), .b(\a[24] ), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n256), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n260), .b(new_n263), .c(new_n223), .d(new_n253), .o1(new_n264));
  nona22aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n259), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n262), .b(new_n265), .o1(\s[24] ));
  norb02aa1n12x5               g171(.a(new_n260), .b(new_n261), .out0(new_n267));
  nano22aa1n12x5               g172(.a(new_n227), .b(new_n252), .c(new_n267), .out0(new_n268));
  oai012aa1n06x5               g173(.a(new_n268), .b(new_n202), .c(new_n198), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n252), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n255), .o1(new_n271));
  inv020aa1n06x5               g176(.a(new_n267), .o1(new_n272));
  oai022aa1n02x5               g177(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n273));
  aob012aa1n02x5               g178(.a(new_n273), .b(\b[23] ), .c(\a[24] ), .out0(new_n274));
  aoai13aa1n12x5               g179(.a(new_n274), .b(new_n272), .c(new_n270), .d(new_n271), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n269), .c(new_n276), .out0(\s[25] ));
  tech160nm_finand02aa1n05x5   g183(.a(new_n269), .b(new_n276), .o1(new_n279));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  tech160nm_fixnrc02aa1n04x5   g185(.a(\b[25] ), .b(\a[26] ), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n279), .d(new_n277), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n277), .b(new_n275), .c(new_n223), .d(new_n268), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n281), .c(new_n280), .out0(new_n284));
  nanp02aa1n03x5               g189(.a(new_n282), .b(new_n284), .o1(\s[26] ));
  norb02aa1n09x5               g190(.a(new_n277), .b(new_n281), .out0(new_n286));
  nano32aa1d12x5               g191(.a(new_n227), .b(new_n286), .c(new_n252), .d(new_n267), .out0(new_n287));
  oai012aa1n12x5               g192(.a(new_n287), .b(new_n202), .c(new_n198), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  oai022aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n290));
  aoi022aa1d18x5               g195(.a(new_n275), .b(new_n286), .c(new_n289), .d(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n291), .out0(\s[27] ));
  xnrc02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nor042aa1n03x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv000aa1n03x5               g200(.a(new_n295), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n292), .o1(new_n297));
  aoai13aa1n06x5               g202(.a(new_n296), .b(new_n297), .c(new_n288), .d(new_n291), .o1(new_n298));
  nanp02aa1n03x5               g203(.a(new_n298), .b(new_n294), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n267), .b(new_n255), .c(new_n233), .d(new_n252), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n286), .o1(new_n301));
  nanp02aa1n02x5               g206(.a(new_n290), .b(new_n289), .o1(new_n302));
  aoai13aa1n04x5               g207(.a(new_n302), .b(new_n301), .c(new_n300), .d(new_n274), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n292), .b(new_n303), .c(new_n223), .d(new_n287), .o1(new_n304));
  nona22aa1n02x5               g209(.a(new_n304), .b(new_n294), .c(new_n295), .out0(new_n305));
  nanp02aa1n03x5               g210(.a(new_n299), .b(new_n305), .o1(\s[28] ));
  norb02aa1n02x5               g211(.a(new_n292), .b(new_n294), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n303), .c(new_n223), .d(new_n287), .o1(new_n308));
  inv000aa1n02x5               g213(.a(new_n307), .o1(new_n309));
  oao003aa1n03x5               g214(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n310));
  aoai13aa1n06x5               g215(.a(new_n310), .b(new_n309), .c(new_n288), .d(new_n291), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n308), .d(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g220(.a(new_n294), .b(new_n292), .c(new_n312), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n303), .c(new_n223), .d(new_n287), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n319));
  inv000aa1n03x5               g224(.a(new_n319), .o1(new_n320));
  aoai13aa1n06x5               g225(.a(new_n320), .b(new_n318), .c(new_n288), .d(new_n291), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norp02aa1n02x5               g227(.a(new_n319), .b(new_n322), .o1(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n317), .d(new_n323), .o1(\s[30] ));
  nano22aa1n06x5               g229(.a(new_n309), .b(new_n312), .c(new_n322), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n303), .c(new_n223), .d(new_n287), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[31] ), .b(\b[30] ), .out0(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n325), .o1(new_n330));
  aoai13aa1n06x5               g235(.a(new_n328), .b(new_n330), .c(new_n288), .d(new_n291), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n327), .c(new_n326), .d(new_n329), .o1(\s[31] ));
  oai012aa1n02x5               g237(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n333), .b(new_n111), .c(new_n112), .out0(\s[3] ));
  oaoi03aa1n02x5               g239(.a(\a[3] ), .b(\b[2] ), .c(new_n333), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g242(.a(new_n115), .b(new_n116), .out0(new_n338));
  norb02aa1n02x5               g243(.a(new_n118), .b(new_n117), .out0(new_n339));
  oai112aa1n02x5               g244(.a(new_n104), .b(new_n339), .c(new_n110), .d(new_n113), .o1(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n338), .b(new_n340), .c(new_n118), .out0(\s[6] ));
  nanb02aa1n02x5               g246(.a(new_n122), .b(new_n123), .out0(new_n342));
  aoai13aa1n02x5               g247(.a(new_n116), .b(new_n115), .c(new_n340), .d(new_n118), .o1(new_n343));
  aoi012aa1n02x5               g248(.a(new_n338), .b(new_n340), .c(new_n118), .o1(new_n344));
  nano23aa1n02x4               g249(.a(new_n344), .b(new_n122), .c(new_n116), .d(new_n123), .out0(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n342), .c(new_n343), .o1(\s[7] ));
  norp02aa1n02x5               g251(.a(new_n345), .b(new_n122), .o1(new_n347));
  xnrb03aa1n02x5               g252(.a(new_n347), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi113aa1n02x5               g253(.a(new_n149), .b(new_n131), .c(new_n126), .d(new_n127), .e(new_n148), .o1(new_n349));
  aboi22aa1n03x5               g254(.a(new_n222), .b(new_n131), .c(new_n349), .d(new_n147), .out0(\s[9] ));
endmodule


