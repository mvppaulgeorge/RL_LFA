// Benchmark "adder" written by ABC on Thu Jul 18 11:18:53 2024

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
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n335, new_n338, new_n339, new_n340,
    new_n342, new_n344, new_n345, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n06x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n03x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .out0(new_n104));
  nor042aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nor002aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n06x4               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nand23aa1n06x5               g015(.a(new_n104), .b(new_n107), .c(new_n110), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\a[3] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[2] ), .o1(new_n113));
  aoai13aa1n12x5               g018(.a(new_n106), .b(new_n105), .c(new_n112), .d(new_n113), .o1(new_n114));
  nand02aa1d06x5               g019(.a(new_n111), .b(new_n114), .o1(new_n115));
  nor002aa1n06x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n04x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norb02aa1n06x4               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nand02aa1d06x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  norp02aa1n24x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nor002aa1n04x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  nand02aa1n06x5               g027(.a(\b[7] ), .b(\a[8] ), .o1(new_n123));
  nona23aa1n02x4               g028(.a(new_n120), .b(new_n123), .c(new_n122), .d(new_n121), .out0(new_n124));
  nano22aa1n03x7               g029(.a(new_n124), .b(new_n119), .c(new_n118), .out0(new_n125));
  nanb03aa1n09x5               g030(.a(new_n116), .b(new_n123), .c(new_n117), .out0(new_n126));
  nor042aa1n03x5               g031(.a(\b[4] ), .b(\a[5] ), .o1(new_n127));
  inv030aa1n04x5               g032(.a(new_n122), .o1(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n121), .c(new_n127), .d(new_n120), .o1(new_n129));
  aob012aa1n12x5               g034(.a(new_n128), .b(new_n116), .c(new_n123), .out0(new_n130));
  oabi12aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n126), .out0(new_n131));
  xorc02aa1n12x5               g036(.a(\a[9] ), .b(\b[8] ), .out0(new_n132));
  aoai13aa1n06x5               g037(.a(new_n132), .b(new_n131), .c(new_n115), .d(new_n125), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n97), .b(new_n133), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand42aa1n08x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1n04x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  oaih22aa1n04x5               g043(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n139), .b(new_n133), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n138), .b(new_n140), .c(new_n135), .out0(\s[11] ));
  nor002aa1d32x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  nand42aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano22aa1n02x4               g049(.a(new_n137), .b(new_n135), .c(new_n136), .out0(new_n145));
  tech160nm_fiaoi012aa1n05x5   g050(.a(new_n137), .b(new_n140), .c(new_n145), .o1(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n143), .c(new_n144), .out0(\s[12] ));
  nano23aa1n06x5               g052(.a(new_n142), .b(new_n137), .c(new_n144), .d(new_n136), .out0(new_n148));
  nand23aa1d12x5               g053(.a(new_n148), .b(new_n97), .c(new_n132), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n131), .c(new_n115), .d(new_n125), .o1(new_n151));
  nanb03aa1n02x5               g056(.a(new_n142), .b(new_n144), .c(new_n136), .out0(new_n152));
  oai112aa1n02x5               g057(.a(new_n139), .b(new_n135), .c(\b[10] ), .d(\a[11] ), .o1(new_n153));
  aob012aa1n03x5               g058(.a(new_n143), .b(new_n137), .c(new_n144), .out0(new_n154));
  oab012aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n152), .out0(new_n155));
  xnrc02aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n155), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  nano23aa1n03x7               g064(.a(new_n122), .b(new_n121), .c(new_n123), .d(new_n120), .out0(new_n160));
  nanp03aa1n03x5               g065(.a(new_n160), .b(new_n118), .c(new_n119), .o1(new_n161));
  oab012aa1n06x5               g066(.a(new_n130), .b(new_n129), .c(new_n126), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n161), .c(new_n111), .d(new_n114), .o1(new_n163));
  oabi12aa1n06x5               g068(.a(new_n154), .b(new_n153), .c(new_n152), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n157), .b(new_n164), .c(new_n163), .d(new_n150), .o1(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n165), .c(new_n159), .out0(\s[14] ));
  nor002aa1n02x5               g072(.a(new_n166), .b(new_n156), .o1(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n163), .d(new_n150), .o1(new_n169));
  inv000aa1d42x5               g074(.a(\b[13] ), .o1(new_n170));
  oai022aa1d18x5               g075(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n171));
  oaib12aa1n06x5               g076(.a(new_n171), .b(new_n170), .c(\a[14] ), .out0(new_n172));
  nor002aa1d32x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n06x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n12x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n169), .c(new_n172), .out0(\s[15] ));
  nanp02aa1n03x5               g082(.a(new_n169), .b(new_n172), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[16] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[15] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n179), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand42aa1n02x5               g087(.a(new_n181), .b(new_n182), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n173), .c(new_n178), .d(new_n174), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n178), .b(new_n176), .o1(new_n185));
  nona22aa1n02x4               g090(.a(new_n185), .b(new_n183), .c(new_n173), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n184), .o1(\s[16] ));
  nor042aa1n04x5               g092(.a(new_n175), .b(new_n183), .o1(new_n188));
  nona22aa1n03x5               g093(.a(new_n188), .b(new_n166), .c(new_n156), .out0(new_n189));
  nor042aa1n06x5               g094(.a(new_n189), .b(new_n149), .o1(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n131), .c(new_n115), .d(new_n125), .o1(new_n191));
  nanp03aa1n02x5               g096(.a(new_n181), .b(new_n174), .c(new_n182), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n179), .b(new_n180), .c(new_n173), .o1(new_n193));
  oai013aa1n02x4               g098(.a(new_n193), .b(new_n192), .c(new_n172), .d(new_n173), .o1(new_n194));
  aoi013aa1n06x4               g099(.a(new_n194), .b(new_n164), .c(new_n168), .d(new_n188), .o1(new_n195));
  nanp02aa1n06x5               g100(.a(new_n191), .b(new_n195), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  aoi113aa1n02x5               g102(.a(new_n194), .b(new_n197), .c(new_n164), .d(new_n168), .e(new_n188), .o1(new_n198));
  aoi022aa1n02x5               g103(.a(new_n196), .b(new_n197), .c(new_n191), .d(new_n198), .o1(\s[17] ));
  inv040aa1d32x5               g104(.a(\a[18] ), .o1(new_n200));
  nor042aa1n03x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  tech160nm_fiaoi012aa1n05x5   g106(.a(new_n201), .b(new_n196), .c(new_n197), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(new_n200), .out0(\s[18] ));
  oabi12aa1n06x5               g108(.a(new_n194), .b(new_n155), .c(new_n189), .out0(new_n204));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  xroi22aa1d06x4               g110(.a(new_n205), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n204), .c(new_n163), .d(new_n190), .o1(new_n207));
  tech160nm_finand02aa1n03p5x5 g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  oab012aa1n02x5               g113(.a(new_n201), .b(\a[18] ), .c(\b[17] ), .out0(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  inv000aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  nor002aa1n04x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n09x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n207), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g121(.a(new_n207), .b(new_n211), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanp02aa1n06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n212), .c(new_n217), .d(new_n213), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n214), .b(new_n210), .c(new_n196), .d(new_n206), .o1(new_n222));
  nona22aa1n02x5               g127(.a(new_n222), .b(new_n220), .c(new_n212), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n221), .b(new_n223), .o1(\s[20] ));
  nanb03aa1n09x5               g129(.a(new_n220), .b(new_n206), .c(new_n214), .out0(new_n225));
  nanb03aa1n06x5               g130(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n226));
  oaih22aa1n04x5               g131(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n227));
  oai112aa1n03x5               g132(.a(new_n227), .b(new_n208), .c(\b[18] ), .d(\a[19] ), .o1(new_n228));
  aoi012aa1n12x5               g133(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n229));
  oai012aa1n12x5               g134(.a(new_n229), .b(new_n228), .c(new_n226), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n04x5               g136(.a(new_n231), .b(new_n225), .c(new_n191), .d(new_n195), .o1(new_n232));
  tech160nm_fixorc02aa1n03p5x5 g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  nano22aa1n03x7               g138(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n234));
  oai012aa1n02x5               g139(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n235));
  norp02aa1n02x5               g140(.a(new_n209), .b(new_n235), .o1(new_n236));
  inv040aa1n03x5               g141(.a(new_n229), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n236), .d(new_n234), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n225), .c(new_n191), .d(new_n195), .o1(new_n239));
  aobi12aa1n02x5               g144(.a(new_n239), .b(new_n233), .c(new_n232), .out0(\s[21] ));
  nor042aa1n04x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  tech160nm_fixnrc02aa1n05x5   g146(.a(\b[21] ), .b(\a[22] ), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n241), .c(new_n232), .d(new_n233), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(new_n232), .b(new_n233), .o1(new_n244));
  nona22aa1n02x4               g149(.a(new_n244), .b(new_n242), .c(new_n241), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n245), .b(new_n243), .o1(\s[22] ));
  xnrc02aa1n02x5               g151(.a(\b[20] ), .b(\a[21] ), .out0(new_n247));
  nor042aa1n02x5               g152(.a(new_n242), .b(new_n247), .o1(new_n248));
  nano32aa1n03x7               g153(.a(new_n220), .b(new_n206), .c(new_n248), .d(new_n214), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n204), .c(new_n163), .d(new_n190), .o1(new_n250));
  nona22aa1n06x5               g155(.a(new_n234), .b(new_n209), .c(new_n235), .out0(new_n251));
  nanb02aa1n06x5               g156(.a(new_n242), .b(new_n233), .out0(new_n252));
  inv000aa1d42x5               g157(.a(\a[22] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(\b[21] ), .o1(new_n254));
  oao003aa1n06x5               g159(.a(new_n253), .b(new_n254), .c(new_n241), .carry(new_n255));
  inv040aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n252), .c(new_n251), .d(new_n229), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[23] ), .b(\b[22] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n250), .c(new_n258), .out0(\s[23] ));
  nand42aa1n02x5               g165(.a(new_n250), .b(new_n258), .o1(new_n261));
  nor002aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  tech160nm_fixnrc02aa1n04x5   g167(.a(\b[23] ), .b(\a[24] ), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n262), .c(new_n261), .d(new_n259), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n259), .b(new_n257), .c(new_n196), .d(new_n249), .o1(new_n265));
  nona22aa1n03x5               g170(.a(new_n265), .b(new_n263), .c(new_n262), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n264), .b(new_n266), .o1(\s[24] ));
  norb02aa1n12x5               g172(.a(new_n259), .b(new_n263), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n225), .b(new_n248), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n204), .c(new_n163), .d(new_n190), .o1(new_n270));
  aoai13aa1n04x5               g175(.a(new_n248), .b(new_n237), .c(new_n236), .d(new_n234), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n268), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\a[24] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(\b[23] ), .o1(new_n274));
  oao003aa1n02x5               g179(.a(new_n273), .b(new_n274), .c(new_n262), .carry(new_n275));
  inv020aa1n04x5               g180(.a(new_n275), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n276), .b(new_n272), .c(new_n271), .d(new_n256), .o1(new_n277));
  nanb02aa1n03x5               g182(.a(new_n277), .b(new_n270), .out0(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  tech160nm_fixorc02aa1n03p5x5 g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  xnrc02aa1n12x5               g186(.a(\b[25] ), .b(\a[26] ), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n278), .d(new_n281), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n281), .b(new_n277), .c(new_n196), .d(new_n269), .o1(new_n284));
  nona22aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n280), .out0(new_n285));
  nanp02aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[26] ));
  norb02aa1n06x5               g191(.a(new_n281), .b(new_n282), .out0(new_n287));
  nano32aa1n03x7               g192(.a(new_n225), .b(new_n287), .c(new_n248), .d(new_n268), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n204), .c(new_n163), .d(new_n190), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .o1(new_n290));
  oai022aa1n02x5               g195(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n291));
  aoi022aa1n06x5               g196(.a(new_n277), .b(new_n287), .c(new_n290), .d(new_n291), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n289), .out0(\s[27] ));
  aoai13aa1n04x5               g199(.a(new_n287), .b(new_n275), .c(new_n257), .d(new_n268), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n291), .b(new_n290), .o1(new_n296));
  nanp03aa1n03x5               g201(.a(new_n289), .b(new_n295), .c(new_n296), .o1(new_n297));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  nor022aa1n06x5               g203(.a(\b[27] ), .b(\a[28] ), .o1(new_n299));
  nanp02aa1n04x5               g204(.a(\b[27] ), .b(\a[28] ), .o1(new_n300));
  nanb02aa1n06x5               g205(.a(new_n299), .b(new_n300), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n297), .d(new_n293), .o1(new_n302));
  aoai13aa1n04x5               g207(.a(new_n268), .b(new_n255), .c(new_n230), .d(new_n248), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n287), .o1(new_n304));
  aoai13aa1n06x5               g209(.a(new_n296), .b(new_n304), .c(new_n303), .d(new_n276), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n293), .b(new_n305), .c(new_n196), .d(new_n288), .o1(new_n306));
  nona22aa1n02x5               g211(.a(new_n306), .b(new_n301), .c(new_n298), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n302), .b(new_n307), .o1(\s[28] ));
  norb02aa1d27x5               g213(.a(new_n293), .b(new_n301), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n305), .c(new_n196), .d(new_n288), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  aoi012aa1n02x5               g216(.a(new_n299), .b(new_n298), .c(new_n300), .o1(new_n312));
  norb02aa1n02x5               g217(.a(new_n312), .b(new_n311), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n309), .o1(new_n314));
  aoai13aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n292), .d(new_n289), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n315), .b(new_n311), .c(new_n310), .d(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d33x5               g222(.a(new_n301), .b(new_n293), .c(new_n311), .out0(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n305), .c(new_n196), .d(new_n288), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .out0(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n320), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n318), .o1(new_n323));
  aoai13aa1n02x5               g228(.a(new_n321), .b(new_n323), .c(new_n292), .d(new_n289), .o1(new_n324));
  aoi022aa1n02x7               g229(.a(new_n324), .b(new_n320), .c(new_n319), .d(new_n322), .o1(\s[30] ));
  nanp03aa1n02x5               g230(.a(new_n309), .b(new_n311), .c(new_n320), .o1(new_n326));
  nanb02aa1n03x5               g231(.a(new_n326), .b(new_n297), .out0(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n329));
  norb02aa1n02x5               g234(.a(new_n329), .b(new_n328), .out0(new_n330));
  aoai13aa1n02x5               g235(.a(new_n329), .b(new_n326), .c(new_n292), .d(new_n289), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n328), .c(new_n327), .d(new_n330), .o1(\s[31] ));
  nanp03aa1n02x5               g237(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n110), .b(new_n333), .c(new_n101), .out0(\s[3] ));
  oaoi03aa1n02x5               g239(.a(new_n112), .b(new_n113), .c(new_n104), .o1(new_n335));
  xnrc02aa1n02x5               g240(.a(new_n335), .b(new_n107), .out0(\s[4] ));
  xnbna2aa1n03x5               g241(.a(new_n119), .b(new_n111), .c(new_n114), .out0(\s[5] ));
  norb02aa1n02x5               g242(.a(new_n120), .b(new_n121), .out0(new_n338));
  aoai13aa1n06x5               g243(.a(new_n338), .b(new_n127), .c(new_n115), .d(new_n119), .o1(new_n339));
  aoi112aa1n02x5               g244(.a(new_n127), .b(new_n338), .c(new_n115), .d(new_n119), .o1(new_n340));
  norb02aa1n02x5               g245(.a(new_n339), .b(new_n340), .out0(\s[6] ));
  inv000aa1d42x5               g246(.a(new_n121), .o1(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n118), .b(new_n339), .c(new_n342), .out0(\s[7] ));
  aobi12aa1n06x5               g248(.a(new_n118), .b(new_n339), .c(new_n342), .out0(new_n344));
  nor042aa1n03x5               g249(.a(new_n344), .b(new_n116), .o1(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n128), .c(new_n123), .out0(\s[8] ));
  aoi112aa1n02x5               g251(.a(new_n132), .b(new_n131), .c(new_n115), .d(new_n125), .o1(new_n347));
  aoi012aa1n02x5               g252(.a(new_n347), .b(new_n163), .c(new_n132), .o1(\s[9] ));
endmodule


