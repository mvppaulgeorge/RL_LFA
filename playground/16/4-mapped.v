// Benchmark "adder" written by ABC on Wed Jul 17 20:09:26 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n345, new_n347, new_n348,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d30x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand42aa1n03x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nor042aa1n09x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  tech160nm_finand02aa1n05x5   g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor002aa1n04x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n02x4               g008(.a(new_n102), .b(new_n100), .c(new_n103), .d(new_n101), .out0(new_n104));
  nanp02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nor022aa1n06x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  norp02aa1n06x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nand42aa1d28x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norb03aa1n02x5               g013(.a(new_n108), .b(new_n106), .c(new_n107), .out0(new_n109));
  nano22aa1n06x5               g014(.a(new_n104), .b(new_n109), .c(new_n105), .out0(new_n110));
  nor042aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanb02aa1n06x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  inv040aa1d30x5               g018(.a(\a[3] ), .o1(new_n114));
  inv040aa1d28x5               g019(.a(\b[2] ), .o1(new_n115));
  nand02aa1d16x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(new_n116), .b(new_n117), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  aoi022aa1d24x5               g024(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n120));
  nor042aa1n03x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oaoi03aa1n12x5               g026(.a(\a[4] ), .b(\b[3] ), .c(new_n116), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n122), .o1(new_n123));
  oai013aa1d12x5               g028(.a(new_n123), .b(new_n121), .c(new_n113), .d(new_n118), .o1(new_n124));
  aoi112aa1n02x5               g029(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n125));
  norb03aa1n03x5               g030(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n126));
  aoi112aa1n03x5               g031(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n127));
  oai112aa1n02x7               g032(.a(new_n126), .b(new_n100), .c(new_n107), .d(new_n127), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n128), .b(new_n125), .c(new_n101), .out0(new_n129));
  xorc02aa1n02x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n110), .o1(new_n131));
  norp02aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n99), .out0(\s[10] ));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand22aa1n12x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  oai112aa1n02x5               g043(.a(new_n131), .b(new_n99), .c(\b[9] ), .d(\a[10] ), .o1(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n138), .b(new_n139), .c(new_n133), .out0(\s[11] ));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1d20x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoi113aa1n02x5               g048(.a(new_n136), .b(new_n143), .c(new_n139), .d(new_n137), .e(new_n133), .o1(new_n144));
  aoi013aa1n02x4               g049(.a(new_n136), .b(new_n139), .c(new_n137), .d(new_n133), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n143), .b(new_n145), .out0(new_n146));
  norp02aa1n02x5               g051(.a(new_n146), .b(new_n144), .o1(\s[12] ));
  and002aa1n03x5               g052(.a(\b[8] ), .b(\a[9] ), .o(new_n148));
  inv030aa1n03x5               g053(.a(new_n137), .o1(new_n149));
  inv000aa1n06x5               g054(.a(new_n133), .o1(new_n150));
  aoi112aa1n09x5               g055(.a(new_n150), .b(new_n132), .c(new_n97), .d(new_n98), .o1(new_n151));
  norb03aa1d15x5               g056(.a(new_n142), .b(new_n136), .c(new_n141), .out0(new_n152));
  nona23aa1d16x5               g057(.a(new_n151), .b(new_n152), .c(new_n149), .d(new_n148), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n129), .c(new_n124), .d(new_n110), .o1(new_n155));
  nona23aa1n09x5               g060(.a(new_n142), .b(new_n137), .c(new_n136), .d(new_n141), .out0(new_n156));
  tech160nm_fioai012aa1n03p5x5 g061(.a(new_n142), .b(new_n141), .c(new_n136), .o1(new_n157));
  aoai13aa1n04x5               g062(.a(new_n133), .b(new_n132), .c(new_n97), .d(new_n98), .o1(new_n158));
  oai012aa1n18x5               g063(.a(new_n157), .b(new_n156), .c(new_n158), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(new_n155), .b(new_n160), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g067(.a(\a[13] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[12] ), .o1(new_n164));
  oaoi03aa1n02x5               g069(.a(new_n163), .b(new_n164), .c(new_n161), .o1(new_n165));
  xnrb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand42aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  aoai13aa1n04x5               g073(.a(new_n168), .b(new_n167), .c(new_n163), .d(new_n164), .o1(new_n169));
  nor022aa1n06x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nand42aa1n04x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  nona23aa1n09x5               g076(.a(new_n168), .b(new_n171), .c(new_n170), .d(new_n167), .out0(new_n172));
  aoai13aa1n02x7               g077(.a(new_n169), .b(new_n172), .c(new_n155), .d(new_n160), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand02aa1d06x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nor022aa1n12x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand02aa1d04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n181));
  aoai13aa1n03x5               g086(.a(new_n180), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n182));
  norb02aa1n02x7               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  nano23aa1n03x5               g088(.a(new_n170), .b(new_n167), .c(new_n168), .d(new_n171), .out0(new_n184));
  nano23aa1n03x7               g089(.a(new_n175), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n185));
  nano22aa1d15x5               g090(.a(new_n153), .b(new_n184), .c(new_n185), .out0(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n129), .c(new_n124), .d(new_n110), .o1(new_n187));
  nona23aa1n09x5               g092(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n177), .out0(new_n188));
  nor002aa1n02x5               g093(.a(new_n188), .b(new_n172), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  oai022aa1n03x5               g095(.a(new_n188), .b(new_n169), .c(\b[15] ), .d(\a[16] ), .o1(new_n191));
  aoi112aa1n09x5               g096(.a(new_n191), .b(new_n190), .c(new_n159), .d(new_n189), .o1(new_n192));
  nand02aa1d08x5               g097(.a(new_n187), .b(new_n192), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  oaoi03aa1n03x5               g101(.a(new_n195), .b(new_n196), .c(new_n193), .o1(new_n197));
  xnrb03aa1n03x5               g102(.a(new_n197), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g103(.a(new_n196), .b(new_n195), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  nor022aa1n06x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand42aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  nano22aa1n12x5               g108(.a(new_n203), .b(new_n199), .c(new_n200), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n202), .b(new_n201), .c(new_n195), .d(new_n196), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n205), .c(new_n187), .d(new_n192), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n03x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nor042aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoi112aa1n03x4               g120(.a(new_n215), .b(new_n210), .c(new_n207), .d(new_n211), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n210), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n211), .b(new_n210), .out0(new_n218));
  nanp02aa1n03x5               g123(.a(new_n207), .b(new_n218), .o1(new_n219));
  tech160nm_fiaoi012aa1n02p5x5 g124(.a(new_n214), .b(new_n219), .c(new_n217), .o1(new_n220));
  norp02aa1n03x5               g125(.a(new_n220), .b(new_n216), .o1(\s[20] ));
  nano23aa1n06x5               g126(.a(new_n210), .b(new_n212), .c(new_n213), .d(new_n211), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n204), .b(new_n222), .o1(new_n223));
  inv000aa1n02x5               g128(.a(new_n206), .o1(new_n224));
  tech160nm_fioaoi03aa1n02p5x5 g129(.a(\a[20] ), .b(\b[19] ), .c(new_n217), .o1(new_n225));
  tech160nm_fiaoi012aa1n05x5   g130(.a(new_n225), .b(new_n222), .c(new_n224), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n223), .c(new_n187), .d(new_n192), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  aoi112aa1n03x4               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n229), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n230), .b(new_n229), .out0(new_n234));
  nand02aa1n02x5               g139(.a(new_n227), .b(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n231), .o1(new_n236));
  tech160nm_fiaoi012aa1n03p5x5 g141(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n237));
  nor042aa1n03x5               g142(.a(new_n237), .b(new_n232), .o1(\s[22] ));
  norp02aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nano23aa1n03x7               g145(.a(new_n229), .b(new_n239), .c(new_n240), .d(new_n230), .out0(new_n241));
  nand23aa1n03x5               g146(.a(new_n204), .b(new_n222), .c(new_n241), .o1(new_n242));
  nona23aa1n02x4               g147(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .out0(new_n243));
  oabi12aa1n06x5               g148(.a(new_n225), .b(new_n243), .c(new_n206), .out0(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[22] ), .b(\b[21] ), .c(new_n233), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n244), .c(new_n241), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n187), .d(new_n192), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  nor042aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  aoi112aa1n03x4               g158(.a(new_n249), .b(new_n253), .c(new_n247), .d(new_n250), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n249), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n250), .b(new_n249), .out0(new_n256));
  nand02aa1n02x5               g161(.a(new_n247), .b(new_n256), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n253), .o1(new_n258));
  tech160nm_fiaoi012aa1n03p5x5 g163(.a(new_n258), .b(new_n257), .c(new_n255), .o1(new_n259));
  nor042aa1n03x5               g164(.a(new_n259), .b(new_n254), .o1(\s[24] ));
  nano23aa1n09x5               g165(.a(new_n249), .b(new_n251), .c(new_n252), .d(new_n250), .out0(new_n261));
  nanb03aa1n02x5               g166(.a(new_n223), .b(new_n261), .c(new_n241), .out0(new_n262));
  oai012aa1n02x5               g167(.a(new_n252), .b(new_n251), .c(new_n249), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n263), .b(new_n261), .c(new_n245), .out0(new_n264));
  inv000aa1n06x5               g169(.a(new_n264), .o1(new_n265));
  aoi013aa1n02x4               g170(.a(new_n265), .b(new_n244), .c(new_n241), .d(new_n261), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n262), .c(new_n187), .d(new_n192), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  aoi112aa1n03x4               g176(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n272));
  inv000aa1n06x5               g177(.a(new_n269), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n270), .b(new_n269), .out0(new_n274));
  nand02aa1n02x5               g179(.a(new_n267), .b(new_n274), .o1(new_n275));
  inv000aa1n03x5               g180(.a(new_n271), .o1(new_n276));
  aoi012aa1n06x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .o1(new_n277));
  nor002aa1n02x5               g182(.a(new_n277), .b(new_n272), .o1(\s[26] ));
  nano23aa1n02x4               g183(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n279));
  nanp03aa1n02x5               g184(.a(new_n279), .b(new_n105), .c(new_n109), .o1(new_n280));
  norb02aa1n02x5               g185(.a(new_n112), .b(new_n111), .out0(new_n281));
  xorc02aa1n02x5               g186(.a(\a[3] ), .b(\b[2] ), .out0(new_n282));
  and002aa1n02x5               g187(.a(\b[0] ), .b(\a[1] ), .o(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[2] ), .b(\b[1] ), .c(new_n283), .o1(new_n284));
  aoi013aa1n02x4               g189(.a(new_n122), .b(new_n284), .c(new_n282), .d(new_n281), .o1(new_n285));
  tech160nm_fiao0012aa1n02p5x5 g190(.a(new_n107), .b(new_n106), .c(new_n108), .o(new_n286));
  aoi112aa1n02x5               g191(.a(new_n125), .b(new_n101), .c(new_n279), .d(new_n286), .o1(new_n287));
  oai012aa1n02x7               g192(.a(new_n287), .b(new_n285), .c(new_n280), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n159), .b(new_n189), .o1(new_n289));
  nona22aa1n02x4               g194(.a(new_n289), .b(new_n191), .c(new_n190), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n276), .b(new_n273), .c(new_n270), .out0(new_n291));
  nano22aa1n03x7               g196(.a(new_n242), .b(new_n291), .c(new_n261), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n290), .c(new_n288), .d(new_n186), .o1(new_n293));
  nano22aa1n03x5               g198(.a(new_n226), .b(new_n241), .c(new_n261), .out0(new_n294));
  oaoi03aa1n12x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .o1(new_n295));
  oaoi13aa1n09x5               g200(.a(new_n295), .b(new_n291), .c(new_n294), .d(new_n265), .o1(new_n296));
  nor042aa1n06x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  nanp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  norb02aa1d27x5               g203(.a(new_n298), .b(new_n297), .out0(new_n299));
  xnbna2aa1n03x5               g204(.a(new_n299), .b(new_n293), .c(new_n296), .out0(\s[27] ));
  inv000aa1n06x5               g205(.a(new_n297), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n299), .o1(new_n302));
  aoi012aa1n02x7               g207(.a(new_n302), .b(new_n293), .c(new_n296), .o1(new_n303));
  xnrc02aa1n12x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n303), .b(new_n301), .c(new_n304), .out0(new_n305));
  nanp03aa1n02x5               g210(.a(new_n244), .b(new_n241), .c(new_n261), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n291), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n295), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n308), .b(new_n307), .c(new_n306), .d(new_n264), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n299), .b(new_n309), .c(new_n193), .d(new_n292), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n304), .b(new_n310), .c(new_n301), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n305), .o1(\s[28] ));
  xnrc02aa1n02x5               g217(.a(\b[28] ), .b(\a[29] ), .out0(new_n313));
  nano22aa1d33x5               g218(.a(new_n304), .b(new_n301), .c(new_n298), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n309), .c(new_n193), .d(new_n292), .o1(new_n315));
  oao003aa1n03x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n313), .b(new_n315), .c(new_n316), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n314), .o1(new_n318));
  aoi012aa1n02x7               g223(.a(new_n318), .b(new_n293), .c(new_n296), .o1(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n313), .c(new_n316), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n317), .b(new_n320), .o1(\s[29] ));
  xnrb03aa1n02x5               g226(.a(new_n283), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1d15x5               g227(.a(new_n313), .b(new_n304), .c(new_n298), .d(new_n301), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n309), .c(new_n193), .d(new_n292), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  tech160nm_fiaoi012aa1n02p5x5 g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n323), .o1(new_n328));
  aoi012aa1n02x7               g233(.a(new_n328), .b(new_n293), .c(new_n296), .o1(new_n329));
  nano22aa1n02x4               g234(.a(new_n329), .b(new_n325), .c(new_n326), .out0(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[30] ));
  xnrc02aa1n02x5               g236(.a(\b[30] ), .b(\a[31] ), .out0(new_n332));
  norb03aa1n02x7               g237(.a(new_n314), .b(new_n313), .c(new_n326), .out0(new_n333));
  inv020aa1n02x5               g238(.a(new_n333), .o1(new_n334));
  aoi012aa1n02x7               g239(.a(new_n334), .b(new_n293), .c(new_n296), .o1(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n336));
  nano22aa1n02x4               g241(.a(new_n335), .b(new_n332), .c(new_n336), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n333), .b(new_n309), .c(new_n193), .d(new_n292), .o1(new_n338));
  tech160nm_fiaoi012aa1n02p5x5 g243(.a(new_n332), .b(new_n338), .c(new_n336), .o1(new_n339));
  norp02aa1n03x5               g244(.a(new_n339), .b(new_n337), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n121), .b(new_n116), .c(new_n117), .out0(\s[3] ));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n121), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n106), .b(new_n124), .c(new_n105), .o1(new_n345));
  xnrb03aa1n02x5               g250(.a(new_n345), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g251(.a(new_n105), .b(new_n106), .out0(new_n347));
  aoi013aa1n02x4               g252(.a(new_n286), .b(new_n124), .c(new_n108), .d(new_n347), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g254(.a(\a[7] ), .b(\b[6] ), .c(new_n348), .o1(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g256(.a(new_n288), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


