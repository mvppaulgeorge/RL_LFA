// Benchmark "adder" written by ABC on Wed Jul 17 17:59:04 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n312, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n335, new_n336, new_n337, new_n339,
    new_n341, new_n342, new_n344, new_n345, new_n346, new_n348, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  oai022aa1n02x5               g001(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n97));
  xnrc02aa1n02x5               g002(.a(\b[2] ), .b(\a[3] ), .out0(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi022aa1d24x5               g004(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n100));
  nor002aa1n03x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  oabi12aa1n06x5               g006(.a(new_n97), .b(new_n101), .c(new_n98), .out0(new_n102));
  nand42aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nor002aa1n06x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nand22aa1n02x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nanb03aa1n06x5               g011(.a(new_n105), .b(new_n106), .c(new_n104), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[8] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[7] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  aoi012aa1n02x7               g015(.a(new_n110), .b(new_n108), .c(new_n109), .o1(new_n111));
  tech160nm_finand02aa1n05x5   g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanb03aa1n12x5               g019(.a(new_n113), .b(new_n114), .c(new_n112), .out0(new_n115));
  nano23aa1n06x5               g020(.a(new_n107), .b(new_n115), .c(new_n111), .d(new_n103), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(new_n109), .b(new_n108), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n113), .o1(new_n118));
  tech160nm_fioai012aa1n03p5x5 g023(.a(new_n112), .b(new_n110), .c(new_n105), .o1(new_n119));
  nand42aa1n02x5               g024(.a(new_n114), .b(new_n106), .o1(new_n120));
  aoai13aa1n06x5               g025(.a(new_n117), .b(new_n120), .c(new_n119), .d(new_n118), .o1(new_n121));
  aoi012aa1n09x5               g026(.a(new_n121), .b(new_n116), .c(new_n102), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  orn002aa1n24x5               g028(.a(\a[10] ), .b(\b[9] ), .o(new_n124));
  nand42aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n124), .b(new_n125), .o1(new_n126));
  tech160nm_fixorc02aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  oai112aa1n06x5               g032(.a(new_n124), .b(new_n125), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  inv040aa1n03x5               g033(.a(new_n128), .o1(new_n129));
  oaib12aa1n02x5               g034(.a(new_n129), .b(new_n122), .c(new_n127), .out0(new_n130));
  aob012aa1n02x5               g035(.a(new_n130), .b(new_n123), .c(new_n126), .out0(\s[10] ));
  nanb02aa1n02x5               g036(.a(new_n126), .b(new_n127), .out0(new_n132));
  obai22aa1n02x7               g037(.a(new_n125), .b(new_n129), .c(new_n122), .d(new_n132), .out0(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nano22aa1n03x7               g041(.a(new_n135), .b(new_n133), .c(new_n136), .out0(new_n137));
  nand22aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  inv000aa1n02x5               g043(.a(new_n138), .o1(new_n139));
  nor002aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  oai022aa1n02x5               g045(.a(new_n137), .b(new_n135), .c(new_n139), .d(new_n140), .o1(new_n141));
  oai022aa1n02x5               g046(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n142));
  oai013aa1n02x4               g047(.a(new_n141), .b(new_n139), .c(new_n137), .d(new_n142), .o1(\s[12] ));
  aoi012aa1n06x5               g048(.a(new_n135), .b(\a[10] ), .c(\b[9] ), .o1(new_n144));
  oai012aa1n06x5               g049(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .o1(new_n145));
  nanb03aa1n12x5               g050(.a(new_n145), .b(new_n144), .c(new_n138), .out0(new_n146));
  nano32aa1n03x7               g051(.a(new_n146), .b(new_n127), .c(new_n124), .d(new_n125), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n121), .c(new_n116), .d(new_n102), .o1(new_n148));
  tech160nm_fioai012aa1n03p5x5 g053(.a(new_n138), .b(new_n140), .c(new_n135), .o1(new_n149));
  oai012aa1n18x5               g054(.a(new_n149), .b(new_n146), .c(new_n129), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  xorc02aa1n03x5               g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(\b[12] ), .b(new_n154), .out0(new_n155));
  xorc02aa1n02x5               g060(.a(\a[3] ), .b(\b[2] ), .out0(new_n156));
  oaoi13aa1n02x5               g061(.a(new_n97), .b(new_n156), .c(new_n100), .d(new_n99), .o1(new_n157));
  nano32aa1n02x4               g062(.a(new_n105), .b(new_n106), .c(new_n103), .d(new_n104), .out0(new_n158));
  nanb03aa1n02x5               g063(.a(new_n115), .b(new_n158), .c(new_n111), .out0(new_n159));
  oabi12aa1n02x5               g064(.a(new_n121), .b(new_n159), .c(new_n157), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n152), .b(new_n150), .c(new_n160), .d(new_n147), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n155), .out0(\s[14] ));
  nanp02aa1n03x5               g068(.a(new_n148), .b(new_n151), .o1(new_n164));
  inv040aa1d32x5               g069(.a(\a[14] ), .o1(new_n165));
  xroi22aa1d06x4               g070(.a(new_n154), .b(\b[12] ), .c(new_n165), .d(\b[13] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(\b[13] ), .o1(new_n167));
  nor042aa1n06x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  oaoi03aa1n12x5               g073(.a(new_n165), .b(new_n167), .c(new_n168), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  tech160nm_fixorc02aa1n03p5x5 g075(.a(\a[15] ), .b(\b[14] ), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n170), .c(new_n164), .d(new_n166), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n170), .c(new_n164), .d(new_n166), .o1(new_n173));
  norb02aa1n02x7               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(\a[15] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(\b[14] ), .b(new_n175), .out0(new_n176));
  xorc02aa1n12x5               g081(.a(\a[16] ), .b(\b[15] ), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  tech160nm_fiaoi012aa1n03p5x5 g083(.a(new_n178), .b(new_n172), .c(new_n176), .o1(new_n179));
  nand43aa1n03x5               g084(.a(new_n172), .b(new_n176), .c(new_n178), .o1(new_n180));
  norb02aa1n03x4               g085(.a(new_n180), .b(new_n179), .out0(\s[16] ));
  inv000aa1d42x5               g086(.a(\a[16] ), .o1(new_n182));
  xroi22aa1d04x5               g087(.a(new_n175), .b(\b[14] ), .c(new_n182), .d(\b[15] ), .out0(new_n183));
  nanp02aa1n03x5               g088(.a(new_n183), .b(new_n166), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n162), .b(new_n152), .o1(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n171), .c(new_n177), .out0(new_n186));
  nand02aa1d04x5               g091(.a(new_n169), .b(new_n176), .o1(new_n187));
  aoi022aa1n02x5               g092(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n187), .b(new_n188), .o1(new_n189));
  oaib12aa1n03x5               g094(.a(new_n189), .b(\b[15] ), .c(new_n182), .out0(new_n190));
  aoi012aa1n09x5               g095(.a(new_n190), .b(new_n150), .c(new_n186), .o1(new_n191));
  oaih12aa1n06x5               g096(.a(new_n191), .b(new_n148), .c(new_n184), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g098(.a(\a[17] ), .o1(new_n194));
  inv040aa1n16x5               g099(.a(\b[16] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  nona23aa1n03x5               g101(.a(new_n128), .b(new_n144), .c(new_n145), .d(new_n139), .out0(new_n197));
  inv000aa1d42x5               g102(.a(\b[15] ), .o1(new_n198));
  aoi022aa1n06x5               g103(.a(new_n187), .b(new_n188), .c(new_n198), .d(new_n182), .o1(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n184), .c(new_n197), .d(new_n149), .o1(new_n200));
  nano22aa1n12x5               g105(.a(new_n122), .b(new_n147), .c(new_n186), .out0(new_n201));
  oai022aa1n03x5               g106(.a(new_n201), .b(new_n200), .c(new_n195), .d(new_n194), .o1(new_n202));
  nor042aa1n03x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nanp02aa1n04x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n196), .out0(\s[18] ));
  nanp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  nano32aa1n03x7               g112(.a(new_n203), .b(new_n196), .c(new_n204), .d(new_n207), .out0(new_n208));
  tech160nm_fioai012aa1n05x5   g113(.a(new_n208), .b(new_n201), .c(new_n200), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n204), .b(new_n203), .c(new_n194), .d(new_n195), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1n04x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n02x5               g121(.a(new_n211), .o1(new_n217));
  tech160nm_fioaoi03aa1n02p5x5 g122(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n192), .d(new_n208), .o1(new_n219));
  nor022aa1n08x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand02aa1n04x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  norb03aa1n02x5               g128(.a(new_n221), .b(new_n211), .c(new_n220), .out0(new_n224));
  aoai13aa1n02x7               g129(.a(new_n224), .b(new_n213), .c(new_n209), .d(new_n210), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n223), .c(new_n219), .d(new_n217), .o1(\s[20] ));
  nano23aa1n06x5               g131(.a(new_n211), .b(new_n220), .c(new_n221), .d(new_n212), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n208), .b(new_n227), .o1(new_n228));
  oabi12aa1n03x5               g133(.a(new_n228), .b(new_n201), .c(new_n200), .out0(new_n229));
  nona23aa1n09x5               g134(.a(new_n221), .b(new_n212), .c(new_n211), .d(new_n220), .out0(new_n230));
  oaoi03aa1n09x5               g135(.a(\a[20] ), .b(\b[19] ), .c(new_n217), .o1(new_n231));
  oabi12aa1n18x5               g136(.a(new_n231), .b(new_n230), .c(new_n210), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[20] ), .b(\a[21] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n229), .c(new_n233), .out0(\s[21] ));
  nona32aa1n02x4               g141(.a(new_n160), .b(new_n184), .c(new_n146), .d(new_n132), .out0(new_n237));
  aoai13aa1n02x5               g142(.a(new_n233), .b(new_n228), .c(new_n237), .d(new_n191), .o1(new_n238));
  nor042aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  tech160nm_fixnrc02aa1n04x5   g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n239), .c(new_n238), .d(new_n235), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n240), .b(new_n239), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n234), .c(new_n229), .d(new_n233), .o1(new_n243));
  nanp02aa1n03x5               g148(.a(new_n241), .b(new_n243), .o1(\s[22] ));
  nor042aa1n06x5               g149(.a(new_n240), .b(new_n234), .o1(new_n245));
  and003aa1n02x5               g150(.a(new_n245), .b(new_n208), .c(new_n227), .o(new_n246));
  oai012aa1n06x5               g151(.a(new_n246), .b(new_n201), .c(new_n200), .o1(new_n247));
  inv020aa1d32x5               g152(.a(\a[22] ), .o1(new_n248));
  inv040aa1d32x5               g153(.a(\b[21] ), .o1(new_n249));
  oao003aa1n12x5               g154(.a(new_n248), .b(new_n249), .c(new_n239), .carry(new_n250));
  aoi012aa1n06x5               g155(.a(new_n250), .b(new_n232), .c(new_n245), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[22] ), .b(\a[23] ), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xnbna2aa1n03x5               g158(.a(new_n253), .b(new_n247), .c(new_n251), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n251), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n253), .b(new_n257), .c(new_n192), .d(new_n246), .o1(new_n258));
  xorc02aa1n03x5               g163(.a(\a[24] ), .b(\b[23] ), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  oai022aa1n02x5               g165(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n260), .b(new_n261), .out0(new_n262));
  aoai13aa1n02x7               g167(.a(new_n262), .b(new_n252), .c(new_n247), .d(new_n251), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n258), .d(new_n256), .o1(\s[24] ));
  nano32aa1n06x5               g169(.a(new_n228), .b(new_n259), .c(new_n245), .d(new_n253), .out0(new_n265));
  oai012aa1n03x5               g170(.a(new_n265), .b(new_n201), .c(new_n200), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n245), .b(new_n231), .c(new_n227), .d(new_n218), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n250), .o1(new_n268));
  norb02aa1n09x5               g173(.a(new_n259), .b(new_n252), .out0(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n261), .b(new_n260), .o1(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n270), .c(new_n267), .d(new_n268), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n266), .c(new_n273), .out0(\s[25] ));
  norp02aa1n02x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n274), .b(new_n272), .c(new_n192), .d(new_n265), .o1(new_n278));
  xorc02aa1n03x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  nanp02aa1n02x5               g184(.a(\b[25] ), .b(\a[26] ), .o1(new_n280));
  oai022aa1n02x5               g185(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n281));
  norb02aa1n02x5               g186(.a(new_n280), .b(new_n281), .out0(new_n282));
  tech160nm_finand02aa1n03p5x5 g187(.a(new_n278), .b(new_n282), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n279), .c(new_n278), .d(new_n277), .o1(\s[26] ));
  nor042aa1n03x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  and002aa1n24x5               g190(.a(\b[26] ), .b(\a[27] ), .o(new_n286));
  norp02aa1n06x5               g191(.a(new_n286), .b(new_n285), .o1(new_n287));
  and002aa1n06x5               g192(.a(new_n279), .b(new_n274), .o(new_n288));
  nano32aa1n06x5               g193(.a(new_n228), .b(new_n288), .c(new_n245), .d(new_n269), .out0(new_n289));
  tech160nm_fioai012aa1n05x5   g194(.a(new_n289), .b(new_n201), .c(new_n200), .o1(new_n290));
  aoi022aa1n12x5               g195(.a(new_n272), .b(new_n288), .c(new_n280), .d(new_n281), .o1(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n287), .b(new_n291), .c(new_n290), .out0(\s[27] ));
  inv000aa1d42x5               g197(.a(new_n286), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n269), .b(new_n250), .c(new_n232), .d(new_n245), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n288), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n281), .b(new_n280), .o1(new_n296));
  aoai13aa1n04x5               g201(.a(new_n296), .b(new_n295), .c(new_n294), .d(new_n271), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n293), .b(new_n297), .c(new_n289), .d(new_n192), .o1(new_n298));
  inv000aa1n03x5               g203(.a(new_n285), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n286), .c(new_n291), .d(new_n290), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n285), .o1(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n298), .d(new_n302), .o1(\s[28] ));
  and002aa1n06x5               g208(.a(new_n301), .b(new_n287), .o(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n297), .c(new_n192), .d(new_n289), .o1(new_n305));
  inv000aa1n02x5               g210(.a(new_n304), .o1(new_n306));
  oao003aa1n03x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n291), .d(new_n290), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n307), .b(new_n309), .out0(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n305), .d(new_n310), .o1(\s[29] ));
  nanp02aa1n02x5               g216(.a(\b[0] ), .b(\a[1] ), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g218(.a(new_n301), .b(new_n309), .c(new_n287), .o(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n297), .c(new_n192), .d(new_n289), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n314), .o1(new_n316));
  oaoi03aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n317));
  inv000aa1n03x5               g222(.a(new_n317), .o1(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n291), .d(new_n290), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .out0(new_n320));
  and002aa1n02x5               g225(.a(\b[28] ), .b(\a[29] ), .o(new_n321));
  oabi12aa1n02x5               g226(.a(new_n320), .b(\a[29] ), .c(\b[28] ), .out0(new_n322));
  oab012aa1n02x4               g227(.a(new_n322), .b(new_n307), .c(new_n321), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n319), .b(new_n320), .c(new_n315), .d(new_n323), .o1(\s[30] ));
  nano22aa1n12x5               g229(.a(new_n306), .b(new_n309), .c(new_n320), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n297), .c(new_n192), .d(new_n289), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[31] ), .b(\b[30] ), .out0(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n325), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n328), .b(new_n330), .c(new_n291), .d(new_n290), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n327), .c(new_n326), .d(new_n329), .o1(\s[31] ));
  inv000aa1d42x5               g237(.a(\a[3] ), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n101), .b(\b[2] ), .c(new_n333), .out0(\s[3] ));
  oai012aa1n02x5               g239(.a(new_n156), .b(new_n100), .c(new_n99), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[4] ), .b(\b[3] ), .out0(new_n336));
  aoib12aa1n02x5               g241(.a(new_n336), .b(new_n333), .c(\b[2] ), .out0(new_n337));
  aoi022aa1n02x5               g242(.a(new_n102), .b(new_n336), .c(new_n335), .d(new_n337), .o1(\s[4] ));
  norb02aa1n02x5               g243(.a(new_n104), .b(new_n105), .out0(new_n339));
  xobna2aa1n03x5               g244(.a(new_n339), .b(new_n102), .c(new_n103), .out0(\s[5] ));
  aoai13aa1n02x5               g245(.a(new_n339), .b(new_n157), .c(\a[4] ), .d(\b[3] ), .o1(new_n341));
  norb02aa1n02x5               g246(.a(new_n112), .b(new_n110), .out0(new_n342));
  xobna2aa1n03x5               g247(.a(new_n342), .b(new_n341), .c(new_n104), .out0(\s[6] ));
  inv000aa1d42x5               g248(.a(new_n110), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n114), .b(new_n113), .out0(new_n345));
  nanp03aa1n02x5               g250(.a(new_n341), .b(new_n104), .c(new_n342), .o1(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n345), .b(new_n346), .c(new_n344), .out0(\s[7] ));
  inv000aa1n02x5               g252(.a(new_n345), .o1(new_n348));
  aoai13aa1n02x5               g253(.a(new_n118), .b(new_n348), .c(new_n346), .d(new_n344), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g255(.a(new_n122), .b(new_n127), .out0(\s[9] ));
endmodule


