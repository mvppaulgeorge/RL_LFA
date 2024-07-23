// Benchmark "adder" written by ABC on Thu Jul 18 11:48:47 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n328, new_n330, new_n331,
    new_n334, new_n336, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n12x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  norp02aa1n24x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nand42aa1d28x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  norb02aa1n02x7               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor002aa1d32x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nanp02aa1n24x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  norb02aa1n06x4               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nor002aa1d24x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand42aa1n16x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor042aa1d18x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n03x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nano22aa1n03x7               g017(.a(new_n112), .b(new_n104), .c(new_n107), .out0(new_n113));
  nand22aa1n12x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  inv000aa1n02x5               g019(.a(new_n114), .o1(new_n115));
  nand22aa1n12x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nand42aa1d28x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nor002aa1n20x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  norb03aa1n03x5               g023(.a(new_n117), .b(new_n116), .c(new_n118), .out0(new_n119));
  nand42aa1d28x5               g024(.a(\b[2] ), .b(\a[3] ), .o1(new_n120));
  nor002aa1d32x5               g025(.a(\b[2] ), .b(\a[3] ), .o1(new_n121));
  nanb03aa1n12x5               g026(.a(new_n121), .b(new_n117), .c(new_n120), .out0(new_n122));
  oab012aa1d24x5               g027(.a(new_n121), .b(\a[4] ), .c(\b[3] ), .out0(new_n123));
  oaoi13aa1n09x5               g028(.a(new_n115), .b(new_n123), .c(new_n119), .d(new_n122), .o1(new_n124));
  inv030aa1n02x5               g029(.a(new_n108), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n110), .b(new_n109), .o1(new_n126));
  tech160nm_fioai012aa1n05x5   g031(.a(new_n103), .b(new_n105), .c(new_n102), .o1(new_n127));
  oai112aa1n03x5               g032(.a(new_n125), .b(new_n126), .c(new_n112), .d(new_n127), .o1(new_n128));
  nand02aa1d24x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norb02aa1n12x5               g034(.a(new_n129), .b(new_n100), .out0(new_n130));
  aoai13aa1n03x5               g035(.a(new_n130), .b(new_n128), .c(new_n124), .d(new_n113), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n99), .b(new_n131), .c(new_n101), .out0(\s[10] ));
  nano23aa1n02x5               g037(.a(new_n102), .b(new_n105), .c(new_n106), .d(new_n103), .out0(new_n133));
  nanb02aa1n12x5               g038(.a(new_n108), .b(new_n109), .out0(new_n134));
  nanb02aa1n03x5               g039(.a(new_n110), .b(new_n111), .out0(new_n135));
  nona22aa1n03x5               g040(.a(new_n133), .b(new_n134), .c(new_n135), .out0(new_n136));
  nona22aa1n09x5               g041(.a(new_n117), .b(new_n118), .c(new_n116), .out0(new_n137));
  nano22aa1n03x7               g042(.a(new_n121), .b(new_n117), .c(new_n120), .out0(new_n138));
  inv000aa1n02x5               g043(.a(new_n123), .o1(new_n139));
  aoai13aa1n04x5               g044(.a(new_n114), .b(new_n139), .c(new_n138), .d(new_n137), .o1(new_n140));
  norp03aa1n02x5               g045(.a(new_n127), .b(new_n135), .c(new_n134), .o1(new_n141));
  nano22aa1n03x7               g046(.a(new_n141), .b(new_n125), .c(new_n126), .out0(new_n142));
  oai012aa1n02x5               g047(.a(new_n142), .b(new_n140), .c(new_n136), .o1(new_n143));
  aoai13aa1n03x5               g048(.a(new_n99), .b(new_n100), .c(new_n143), .d(new_n129), .o1(new_n144));
  oai012aa1n06x5               g049(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n145));
  nor002aa1d24x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand22aa1n12x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n145), .out0(\s[11] ));
  nanp02aa1n03x5               g054(.a(new_n131), .b(new_n101), .o1(new_n150));
  inv040aa1n03x5               g055(.a(new_n145), .o1(new_n151));
  aoai13aa1n03x5               g056(.a(new_n148), .b(new_n151), .c(new_n150), .d(new_n99), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nand22aa1n12x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  nona22aa1n02x5               g060(.a(new_n152), .b(new_n155), .c(new_n146), .out0(new_n156));
  inv000aa1n02x5               g061(.a(new_n155), .o1(new_n157));
  oaoi13aa1n02x7               g062(.a(new_n157), .b(new_n152), .c(\a[11] ), .d(\b[10] ), .o1(new_n158));
  norb02aa1n03x4               g063(.a(new_n156), .b(new_n158), .out0(\s[12] ));
  tech160nm_fioai012aa1n05x5   g064(.a(new_n123), .b(new_n119), .c(new_n122), .o1(new_n160));
  nanp03aa1n03x5               g065(.a(new_n113), .b(new_n160), .c(new_n114), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n146), .b(new_n153), .c(new_n154), .d(new_n147), .out0(new_n162));
  nanp03aa1n03x5               g067(.a(new_n162), .b(new_n99), .c(new_n130), .o1(new_n163));
  nona23aa1n09x5               g068(.a(new_n154), .b(new_n147), .c(new_n146), .d(new_n153), .out0(new_n164));
  ao0012aa1n03x5               g069(.a(new_n153), .b(new_n146), .c(new_n154), .o(new_n165));
  oabi12aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n145), .out0(new_n166));
  inv040aa1n03x5               g071(.a(new_n166), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n142), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d28x5               g074(.a(\a[14] ), .o1(new_n170));
  nor042aa1d18x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  xnrc02aa1n12x5               g076(.a(\b[12] ), .b(\a[13] ), .out0(new_n172));
  aoib12aa1n03x5               g077(.a(new_n171), .b(new_n168), .c(new_n172), .out0(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[13] ), .c(new_n170), .out0(\s[14] ));
  xnrc02aa1n06x5               g079(.a(\b[13] ), .b(\a[14] ), .out0(new_n175));
  nor042aa1n03x5               g080(.a(new_n175), .b(new_n172), .o1(new_n176));
  inv040aa1d32x5               g081(.a(\b[13] ), .o1(new_n177));
  oao003aa1n06x5               g082(.a(new_n170), .b(new_n177), .c(new_n171), .carry(new_n178));
  nor002aa1d32x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand42aa1n08x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n178), .c(new_n168), .d(new_n176), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n182), .b(new_n178), .c(new_n168), .d(new_n176), .o1(new_n184));
  norb02aa1n02x7               g089(.a(new_n183), .b(new_n184), .out0(\s[15] ));
  nor002aa1d32x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nand02aa1n06x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  oai112aa1n02x7               g093(.a(new_n183), .b(new_n188), .c(\b[14] ), .d(\a[15] ), .o1(new_n189));
  oaoi13aa1n06x5               g094(.a(new_n188), .b(new_n183), .c(\a[15] ), .d(\b[14] ), .o1(new_n190));
  norb02aa1n03x4               g095(.a(new_n189), .b(new_n190), .out0(\s[16] ));
  nano23aa1n03x7               g096(.a(new_n179), .b(new_n186), .c(new_n187), .d(new_n180), .out0(new_n192));
  nano22aa1n03x7               g097(.a(new_n163), .b(new_n176), .c(new_n192), .out0(new_n193));
  aoai13aa1n03x5               g098(.a(new_n193), .b(new_n128), .c(new_n124), .d(new_n113), .o1(new_n194));
  aoai13aa1n04x5               g099(.a(new_n192), .b(new_n178), .c(new_n166), .d(new_n176), .o1(new_n195));
  aoi012aa1n09x5               g100(.a(new_n186), .b(new_n179), .c(new_n187), .o1(new_n196));
  nand23aa1n04x5               g101(.a(new_n194), .b(new_n195), .c(new_n196), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g103(.a(\a[17] ), .o1(new_n199));
  inv040aa1d28x5               g104(.a(\b[16] ), .o1(new_n200));
  nand42aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  nano22aa1n03x7               g106(.a(new_n164), .b(new_n99), .c(new_n130), .out0(new_n202));
  nona23aa1d18x5               g107(.a(new_n187), .b(new_n180), .c(new_n179), .d(new_n186), .out0(new_n203));
  nona32aa1n09x5               g108(.a(new_n202), .b(new_n203), .c(new_n175), .d(new_n172), .out0(new_n204));
  oaoi13aa1n12x5               g109(.a(new_n204), .b(new_n142), .c(new_n140), .d(new_n136), .o1(new_n205));
  inv030aa1n02x5               g110(.a(new_n178), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n176), .b(new_n165), .c(new_n162), .d(new_n151), .o1(new_n207));
  aoai13aa1n12x5               g112(.a(new_n196), .b(new_n203), .c(new_n207), .d(new_n206), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  oai112aa1n03x5               g114(.a(new_n201), .b(new_n209), .c(new_n208), .d(new_n205), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nand42aa1d28x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nanb02aa1n12x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n201), .out0(\s[18] ));
  nano22aa1n03x7               g119(.a(new_n213), .b(new_n201), .c(new_n209), .out0(new_n215));
  tech160nm_fioai012aa1n03p5x5 g120(.a(new_n215), .b(new_n208), .c(new_n205), .o1(new_n216));
  aoai13aa1n12x5               g121(.a(new_n212), .b(new_n211), .c(new_n199), .d(new_n200), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand22aa1n09x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  xobna2aa1n03x5               g125(.a(new_n220), .b(new_n216), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g127(.a(new_n218), .o1(new_n223));
  tech160nm_fiaoi012aa1n02p5x5 g128(.a(new_n220), .b(new_n216), .c(new_n217), .o1(new_n224));
  nor002aa1d24x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand22aa1n09x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(new_n227));
  nano22aa1n02x4               g132(.a(new_n224), .b(new_n223), .c(new_n227), .out0(new_n228));
  inv040aa1n02x5               g133(.a(new_n217), .o1(new_n229));
  oaoi13aa1n03x5               g134(.a(new_n229), .b(new_n215), .c(new_n208), .d(new_n205), .o1(new_n230));
  oaoi13aa1n02x7               g135(.a(new_n227), .b(new_n223), .c(new_n230), .d(new_n220), .o1(new_n231));
  norp02aa1n03x5               g136(.a(new_n231), .b(new_n228), .o1(\s[20] ));
  nano23aa1n06x5               g137(.a(new_n218), .b(new_n225), .c(new_n226), .d(new_n219), .out0(new_n233));
  nand02aa1d04x5               g138(.a(new_n215), .b(new_n233), .o1(new_n234));
  inv000aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  nona23aa1n09x5               g140(.a(new_n226), .b(new_n219), .c(new_n218), .d(new_n225), .out0(new_n236));
  tech160nm_fiaoi012aa1n04x5   g141(.a(new_n225), .b(new_n218), .c(new_n226), .o1(new_n237));
  tech160nm_fioai012aa1n05x5   g142(.a(new_n237), .b(new_n236), .c(new_n217), .o1(new_n238));
  oaoi13aa1n06x5               g143(.a(new_n238), .b(new_n235), .c(new_n208), .d(new_n205), .o1(new_n239));
  xnrb03aa1n03x5               g144(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  inv040aa1n03x5               g146(.a(new_n241), .o1(new_n242));
  oaih12aa1n02x5               g147(.a(new_n235), .b(new_n208), .c(new_n205), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  aoib12aa1n02x7               g149(.a(new_n244), .b(new_n243), .c(new_n238), .out0(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  nano22aa1n03x5               g151(.a(new_n245), .b(new_n242), .c(new_n246), .out0(new_n247));
  oaoi13aa1n02x7               g152(.a(new_n246), .b(new_n242), .c(new_n239), .d(new_n244), .o1(new_n248));
  norp02aa1n03x5               g153(.a(new_n248), .b(new_n247), .o1(\s[22] ));
  nor042aa1n03x5               g154(.a(new_n246), .b(new_n244), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n234), .out0(new_n251));
  tech160nm_fioai012aa1n03p5x5 g156(.a(new_n251), .b(new_n208), .c(new_n205), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .carry(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  aoi012aa1n02x7               g159(.a(new_n254), .b(new_n238), .c(new_n250), .o1(new_n255));
  xnrc02aa1n12x5               g160(.a(\b[22] ), .b(\a[23] ), .out0(new_n256));
  xobna2aa1n03x5               g161(.a(new_n256), .b(new_n252), .c(new_n255), .out0(\s[23] ));
  nor042aa1n09x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  tech160nm_fiaoi012aa1n02p5x5 g164(.a(new_n256), .b(new_n252), .c(new_n255), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[23] ), .b(\a[24] ), .out0(new_n261));
  nano22aa1n03x5               g166(.a(new_n260), .b(new_n259), .c(new_n261), .out0(new_n262));
  inv000aa1n02x5               g167(.a(new_n255), .o1(new_n263));
  oaoi13aa1n03x5               g168(.a(new_n263), .b(new_n251), .c(new_n208), .d(new_n205), .o1(new_n264));
  oaoi13aa1n02x7               g169(.a(new_n261), .b(new_n259), .c(new_n264), .d(new_n256), .o1(new_n265));
  norp02aa1n03x5               g170(.a(new_n265), .b(new_n262), .o1(\s[24] ));
  nor042aa1n02x5               g171(.a(new_n261), .b(new_n256), .o1(new_n267));
  nano22aa1n02x4               g172(.a(new_n234), .b(new_n250), .c(new_n267), .out0(new_n268));
  inv000aa1n02x5               g173(.a(new_n237), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n250), .b(new_n269), .c(new_n233), .d(new_n229), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n267), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n270), .d(new_n253), .o1(new_n273));
  oaoi13aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n208), .d(new_n205), .o1(new_n274));
  xnrb03aa1n03x5               g179(.a(new_n274), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n268), .b(new_n208), .c(new_n205), .o1(new_n278));
  tech160nm_fixnrc02aa1n05x5   g183(.a(\b[24] ), .b(\a[25] ), .out0(new_n279));
  aoib12aa1n02x7               g184(.a(new_n279), .b(new_n278), .c(new_n273), .out0(new_n280));
  xnrc02aa1n12x5               g185(.a(\b[25] ), .b(\a[26] ), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n280), .b(new_n277), .c(new_n281), .out0(new_n282));
  oaoi13aa1n02x7               g187(.a(new_n281), .b(new_n277), .c(new_n274), .d(new_n279), .o1(new_n283));
  norp02aa1n03x5               g188(.a(new_n283), .b(new_n282), .o1(\s[26] ));
  nor042aa1n03x5               g189(.a(new_n281), .b(new_n279), .o1(new_n285));
  nano32aa1n03x7               g190(.a(new_n234), .b(new_n285), .c(new_n250), .d(new_n267), .out0(new_n286));
  oai012aa1n12x5               g191(.a(new_n286), .b(new_n208), .c(new_n205), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n277), .carry(new_n288));
  aobi12aa1n06x5               g193(.a(new_n288), .b(new_n273), .c(new_n285), .out0(new_n289));
  xorc02aa1n12x5               g194(.a(\a[27] ), .b(\b[26] ), .out0(new_n290));
  xnbna2aa1n03x5               g195(.a(new_n290), .b(new_n289), .c(new_n287), .out0(\s[27] ));
  nor042aa1n03x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv040aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  aobi12aa1n06x5               g198(.a(new_n290), .b(new_n289), .c(new_n287), .out0(new_n294));
  tech160nm_fixnrc02aa1n04x5   g199(.a(\b[27] ), .b(\a[28] ), .out0(new_n295));
  nano22aa1n03x7               g200(.a(new_n294), .b(new_n293), .c(new_n295), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n267), .b(new_n254), .c(new_n238), .d(new_n250), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n285), .o1(new_n298));
  aoai13aa1n06x5               g203(.a(new_n288), .b(new_n298), .c(new_n297), .d(new_n272), .o1(new_n299));
  aoai13aa1n02x5               g204(.a(new_n290), .b(new_n299), .c(new_n197), .d(new_n286), .o1(new_n300));
  aoi012aa1n02x7               g205(.a(new_n295), .b(new_n300), .c(new_n293), .o1(new_n301));
  norp02aa1n03x5               g206(.a(new_n301), .b(new_n296), .o1(\s[28] ));
  norb02aa1n02x5               g207(.a(new_n290), .b(new_n295), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n299), .c(new_n197), .d(new_n286), .o1(new_n304));
  oao003aa1n03x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n305));
  xnrc02aa1n12x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n06x5               g212(.a(new_n303), .b(new_n289), .c(new_n287), .out0(new_n308));
  nano22aa1n03x7               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g216(.a(new_n290), .b(new_n306), .c(new_n295), .out0(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n299), .c(new_n197), .d(new_n286), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n06x5               g221(.a(new_n312), .b(new_n289), .c(new_n287), .out0(new_n317));
  nano22aa1n03x7               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  xnrc02aa1n02x5               g224(.a(\b[30] ), .b(\a[31] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n312), .b(new_n315), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n299), .c(new_n197), .d(new_n286), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n320), .b(new_n322), .c(new_n323), .o1(new_n324));
  aobi12aa1n06x5               g229(.a(new_n321), .b(new_n289), .c(new_n287), .out0(new_n325));
  nano22aa1n03x7               g230(.a(new_n325), .b(new_n320), .c(new_n323), .out0(new_n326));
  norp02aa1n03x5               g231(.a(new_n324), .b(new_n326), .o1(\s[31] ));
  nanb02aa1n02x5               g232(.a(new_n121), .b(new_n120), .out0(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n328), .b(new_n137), .c(new_n117), .out0(\s[3] ));
  xorc02aa1n02x5               g234(.a(\a[4] ), .b(\b[3] ), .out0(new_n330));
  aoi112aa1n02x5               g235(.a(new_n330), .b(new_n121), .c(new_n138), .d(new_n137), .o1(new_n331));
  oaoi13aa1n02x5               g236(.a(new_n331), .b(new_n124), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xobna2aa1n03x5               g237(.a(new_n107), .b(new_n160), .c(new_n114), .out0(\s[5] ));
  aoi013aa1n02x4               g238(.a(new_n105), .b(new_n160), .c(new_n114), .d(new_n106), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(new_n334), .b(new_n104), .out0(\s[6] ));
  aobi12aa1n03x5               g240(.a(new_n127), .b(new_n124), .c(new_n133), .out0(new_n336));
  xnrb03aa1n02x5               g241(.a(new_n336), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g242(.a(\a[7] ), .b(\b[6] ), .c(new_n336), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g244(.a(new_n130), .b(new_n161), .c(new_n142), .out0(\s[9] ));
endmodule


