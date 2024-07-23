// Benchmark "adder" written by ABC on Thu Jul 18 06:39:44 2024

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
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n327,
    new_n328, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nand02aa1d06x5               g002(.a(\b[5] ), .b(\a[6] ), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[5] ), .b(\a[6] ), .o1(new_n99));
  nor042aa1d18x5               g004(.a(\b[4] ), .b(\a[5] ), .o1(new_n100));
  nand02aa1d08x5               g005(.a(\b[4] ), .b(\a[5] ), .o1(new_n101));
  nona23aa1d24x5               g006(.a(new_n98), .b(new_n101), .c(new_n100), .d(new_n99), .out0(new_n102));
  nor042aa1d18x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nand22aa1n03x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nor042aa1n02x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nona23aa1d18x5               g011(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n103), .out0(new_n107));
  nor042aa1n12x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  nor042aa1n03x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand22aa1n04x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  nand22aa1n03x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  aoi012aa1n06x5               g016(.a(new_n109), .b(new_n110), .c(new_n111), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[4] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(\b[3] ), .b(new_n114), .out0(new_n115));
  nand02aa1n04x5               g020(.a(new_n115), .b(new_n113), .o1(new_n116));
  inv040aa1d32x5               g021(.a(\a[3] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\b[2] ), .o1(new_n118));
  nand02aa1d12x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[2] ), .b(\a[3] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n119), .b(new_n120), .o1(new_n121));
  oaoi03aa1n09x5               g026(.a(\a[4] ), .b(\b[3] ), .c(new_n119), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  oai013aa1n03x5               g028(.a(new_n123), .b(new_n112), .c(new_n116), .d(new_n121), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n103), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n106), .b(new_n104), .o1(new_n126));
  aoi012aa1n06x5               g031(.a(new_n99), .b(new_n100), .c(new_n98), .o1(new_n127));
  oai112aa1n06x5               g032(.a(new_n125), .b(new_n126), .c(new_n107), .d(new_n127), .o1(new_n128));
  tech160nm_fixorc02aa1n04x5   g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n124), .d(new_n108), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n09x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n03x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n97), .out0(\s[10] ));
  nor022aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n12x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  nor002aa1d32x5               g042(.a(\b[8] ), .b(\a[9] ), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n130), .b(new_n131), .c(new_n138), .out0(new_n139));
  xobna2aa1n03x5               g044(.a(new_n137), .b(new_n139), .c(new_n132), .out0(\s[11] ));
  aoi013aa1n02x4               g045(.a(new_n135), .b(new_n139), .c(new_n137), .d(new_n132), .o1(new_n141));
  nor022aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  norb02aa1n03x5               g049(.a(new_n144), .b(new_n141), .out0(new_n145));
  aoi113aa1n02x5               g050(.a(new_n135), .b(new_n144), .c(new_n139), .d(new_n136), .e(new_n132), .o1(new_n146));
  norp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(\s[12] ));
  nor043aa1n02x5               g052(.a(new_n112), .b(new_n116), .c(new_n121), .o1(new_n148));
  oai012aa1n04x7               g053(.a(new_n108), .b(new_n148), .c(new_n122), .o1(new_n149));
  nor042aa1n02x5               g054(.a(new_n107), .b(new_n127), .o1(new_n150));
  nano22aa1n03x7               g055(.a(new_n150), .b(new_n125), .c(new_n126), .out0(new_n151));
  nano23aa1n02x4               g056(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n152));
  nanp03aa1n02x5               g057(.a(new_n152), .b(new_n129), .c(new_n133), .o1(new_n153));
  oai112aa1n06x5               g058(.a(new_n132), .b(new_n136), .c(new_n131), .d(new_n138), .o1(new_n154));
  nor042aa1n06x5               g059(.a(new_n142), .b(new_n135), .o1(new_n155));
  aoi022aa1d18x5               g060(.a(new_n154), .b(new_n155), .c(\b[11] ), .d(\a[12] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n153), .c(new_n149), .d(new_n151), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n09x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  tech160nm_finand02aa1n03p5x5 g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nona23aa1n02x4               g068(.a(new_n143), .b(new_n136), .c(new_n135), .d(new_n142), .out0(new_n164));
  nano22aa1n03x5               g069(.a(new_n164), .b(new_n129), .c(new_n133), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n128), .c(new_n124), .d(new_n108), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand22aa1n09x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nona23aa1n09x5               g073(.a(new_n168), .b(new_n161), .c(new_n160), .d(new_n167), .out0(new_n169));
  aoi012aa1n02x5               g074(.a(new_n167), .b(new_n160), .c(new_n168), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n169), .c(new_n166), .d(new_n157), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  nano23aa1n02x4               g079(.a(new_n160), .b(new_n167), .c(new_n168), .d(new_n161), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n170), .o1(new_n176));
  nand42aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanb02aa1n18x5               g082(.a(new_n173), .b(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n176), .c(new_n158), .d(new_n175), .o1(new_n180));
  nor022aa1n08x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanp02aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanb02aa1d24x5               g087(.a(new_n181), .b(new_n182), .out0(new_n183));
  aoi012aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n174), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n183), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n173), .b(new_n185), .c(new_n171), .d(new_n177), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n184), .b(new_n186), .o1(\s[16] ));
  nor043aa1d12x5               g092(.a(new_n169), .b(new_n178), .c(new_n183), .o1(new_n188));
  nand02aa1n02x5               g093(.a(new_n165), .b(new_n188), .o1(new_n189));
  aoai13aa1n02x7               g094(.a(new_n177), .b(new_n167), .c(new_n160), .d(new_n168), .o1(new_n190));
  nor002aa1n02x5               g095(.a(new_n181), .b(new_n173), .o1(new_n191));
  aoi022aa1n06x5               g096(.a(new_n190), .b(new_n191), .c(\b[15] ), .d(\a[16] ), .o1(new_n192));
  aoi012aa1d24x5               g097(.a(new_n192), .b(new_n188), .c(new_n156), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n189), .c(new_n149), .d(new_n151), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n03x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  xnrc02aa1n12x5               g101(.a(\b[16] ), .b(\a[17] ), .out0(new_n197));
  aoib12aa1n06x5               g102(.a(new_n196), .b(new_n194), .c(new_n197), .out0(new_n198));
  xnrb03aa1n02x5               g103(.a(new_n198), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nano32aa1n03x7               g104(.a(new_n153), .b(new_n185), .c(new_n175), .d(new_n179), .out0(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n128), .c(new_n108), .d(new_n124), .o1(new_n201));
  nor002aa1n06x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand22aa1n04x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb03aa1d15x5               g108(.a(new_n203), .b(new_n197), .c(new_n202), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoi012aa1n06x5               g110(.a(new_n202), .b(new_n196), .c(new_n203), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n205), .c(new_n201), .d(new_n193), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  tech160nm_finand02aa1n05x5   g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  xnrc02aa1n12x5               g116(.a(\b[19] ), .b(\a[20] ), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n210), .b(new_n213), .c(new_n207), .d(new_n211), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n210), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n206), .o1(new_n216));
  nanb02aa1n18x5               g121(.a(new_n210), .b(new_n211), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n02x7               g123(.a(new_n218), .b(new_n216), .c(new_n194), .d(new_n204), .o1(new_n219));
  aoi012aa1n03x5               g124(.a(new_n212), .b(new_n219), .c(new_n215), .o1(new_n220));
  norp02aa1n02x5               g125(.a(new_n220), .b(new_n214), .o1(\s[20] ));
  nona22aa1d24x5               g126(.a(new_n204), .b(new_n217), .c(new_n212), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n211), .b(new_n202), .c(new_n196), .d(new_n203), .o1(new_n223));
  oab012aa1n04x5               g128(.a(new_n210), .b(\a[20] ), .c(\b[19] ), .out0(new_n224));
  aoi022aa1n12x5               g129(.a(new_n223), .b(new_n224), .c(\b[19] ), .d(\a[20] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n02x7               g131(.a(new_n226), .b(new_n222), .c(new_n201), .d(new_n193), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n06x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  nand42aa1n03x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  nor022aa1n06x5               g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  tech160nm_finand02aa1n03p5x5 g137(.a(\b[21] ), .b(\a[22] ), .o1(new_n233));
  nanb02aa1n12x5               g138(.a(new_n232), .b(new_n233), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n229), .b(new_n235), .c(new_n227), .d(new_n231), .o1(new_n236));
  inv030aa1n02x5               g141(.a(new_n229), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n222), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n231), .b(new_n225), .c(new_n194), .d(new_n238), .o1(new_n239));
  aoi012aa1n03x5               g144(.a(new_n234), .b(new_n239), .c(new_n237), .o1(new_n240));
  norp02aa1n03x5               g145(.a(new_n240), .b(new_n236), .o1(\s[22] ));
  nano23aa1n06x5               g146(.a(new_n229), .b(new_n232), .c(new_n233), .d(new_n230), .out0(new_n242));
  nona23aa1d16x5               g147(.a(new_n204), .b(new_n242), .c(new_n212), .d(new_n217), .out0(new_n243));
  nand02aa1n06x5               g148(.a(new_n223), .b(new_n224), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(\b[19] ), .b(\a[20] ), .o1(new_n245));
  nano32aa1n03x7               g150(.a(new_n234), .b(new_n237), .c(new_n230), .d(new_n245), .out0(new_n246));
  oaoi03aa1n12x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .o1(new_n247));
  aoi012aa1n09x5               g152(.a(new_n247), .b(new_n244), .c(new_n246), .o1(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n243), .c(new_n201), .d(new_n193), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  tech160nm_fixorc02aa1n02p5x5 g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n251), .b(new_n253), .c(new_n249), .d(new_n252), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n251), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n243), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n248), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n252), .b(new_n257), .c(new_n194), .d(new_n256), .o1(new_n258));
  aobi12aa1n03x5               g163(.a(new_n253), .b(new_n258), .c(new_n255), .out0(new_n259));
  norp02aa1n02x5               g164(.a(new_n259), .b(new_n254), .o1(\s[24] ));
  and002aa1n02x5               g165(.a(new_n253), .b(new_n252), .o(new_n261));
  nano22aa1n03x7               g166(.a(new_n222), .b(new_n261), .c(new_n242), .out0(new_n262));
  inv030aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n261), .b(new_n247), .c(new_n244), .d(new_n246), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .c(new_n255), .carry(new_n265));
  nanp02aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(new_n266));
  inv020aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n263), .c(new_n201), .d(new_n193), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  xorc02aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  aoi112aa1n02x5               g177(.a(new_n270), .b(new_n272), .c(new_n268), .d(new_n271), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n270), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n271), .b(new_n266), .c(new_n194), .d(new_n262), .o1(new_n275));
  aobi12aa1n02x5               g180(.a(new_n272), .b(new_n275), .c(new_n274), .out0(new_n276));
  nor002aa1n02x5               g181(.a(new_n276), .b(new_n273), .o1(\s[26] ));
  nano32aa1d12x5               g182(.a(new_n243), .b(new_n272), .c(new_n261), .d(new_n271), .out0(new_n278));
  nanp02aa1n02x5               g183(.a(new_n272), .b(new_n271), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .carry(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n264), .d(new_n265), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n194), .d(new_n278), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(new_n281), .b(new_n282), .c(new_n194), .d(new_n278), .o1(new_n284));
  norb02aa1n02x5               g189(.a(new_n283), .b(new_n284), .out0(\s[27] ));
  nor042aa1n04x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[28] ), .b(\b[27] ), .out0(new_n287));
  nona22aa1n02x5               g192(.a(new_n283), .b(new_n287), .c(new_n286), .out0(new_n288));
  inv000aa1n03x5               g193(.a(new_n286), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n283), .c(new_n289), .o1(new_n291));
  norb02aa1n03x4               g196(.a(new_n288), .b(new_n291), .out0(\s[28] ));
  xorc02aa1n06x5               g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  and002aa1n02x5               g199(.a(new_n287), .b(new_n282), .o(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n281), .c(new_n194), .d(new_n278), .o1(new_n296));
  oao003aa1n09x5               g201(.a(\a[28] ), .b(\b[27] ), .c(new_n289), .carry(new_n297));
  tech160nm_fiaoi012aa1n05x5   g202(.a(new_n294), .b(new_n296), .c(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  nona22aa1n03x5               g204(.a(new_n296), .b(new_n299), .c(new_n293), .out0(new_n300));
  norb02aa1n03x4               g205(.a(new_n300), .b(new_n298), .out0(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g207(.a(new_n294), .b(new_n282), .c(new_n287), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n281), .c(new_n194), .d(new_n278), .o1(new_n304));
  tech160nm_fioaoi03aa1n03p5x5 g209(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n305));
  inv000aa1n03x5               g210(.a(new_n305), .o1(new_n306));
  tech160nm_fixorc02aa1n05x5   g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  aoi012aa1n06x5               g213(.a(new_n308), .b(new_n304), .c(new_n306), .o1(new_n309));
  nona22aa1n03x5               g214(.a(new_n304), .b(new_n305), .c(new_n307), .out0(new_n310));
  norb02aa1n03x4               g215(.a(new_n310), .b(new_n309), .out0(\s[30] ));
  nano32aa1n02x4               g216(.a(new_n308), .b(new_n293), .c(new_n287), .d(new_n282), .out0(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n281), .c(new_n194), .d(new_n278), .o1(new_n313));
  oao003aa1n06x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n314));
  inv000aa1n02x5               g219(.a(new_n314), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[31] ), .b(\b[30] ), .out0(new_n316));
  nona22aa1n03x5               g221(.a(new_n313), .b(new_n315), .c(new_n316), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n318), .b(new_n313), .c(new_n314), .o1(new_n319));
  norb02aa1n03x4               g224(.a(new_n317), .b(new_n319), .out0(\s[31] ));
  xnbna2aa1n03x5               g225(.a(new_n112), .b(new_n120), .c(new_n119), .out0(\s[3] ));
  oaoi03aa1n02x5               g226(.a(\a[3] ), .b(\b[2] ), .c(new_n112), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g229(.a(new_n100), .b(new_n124), .c(new_n101), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g231(.a(new_n102), .o1(new_n327));
  aobi12aa1n02x5               g232(.a(new_n127), .b(new_n124), .c(new_n327), .out0(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g234(.a(\a[7] ), .b(\b[6] ), .c(new_n328), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g236(.a(new_n129), .b(new_n149), .c(new_n151), .out0(\s[9] ));
endmodule


