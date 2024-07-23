// Benchmark "adder" written by ABC on Thu Jul 18 09:49:51 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n333, new_n335, new_n336, new_n337, new_n338, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  tech160nm_finand02aa1n03p5x5 g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nona22aa1n02x4               g007(.a(new_n101), .b(new_n102), .c(new_n100), .out0(new_n103));
  nor042aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano22aa1n02x4               g010(.a(new_n104), .b(new_n101), .c(new_n105), .out0(new_n106));
  inv000aa1d42x5               g011(.a(\a[4] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[3] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n104), .b(new_n107), .c(new_n108), .o1(new_n109));
  aobi12aa1n06x5               g014(.a(new_n109), .b(new_n106), .c(new_n103), .out0(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanb02aa1n02x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nand42aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor002aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  xnrc02aa1n12x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  inv000aa1d42x5               g022(.a(new_n117), .o1(new_n118));
  orn002aa1n03x5               g023(.a(\a[6] ), .b(\b[5] ), .o(new_n119));
  nand42aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  oai112aa1n03x5               g025(.a(new_n119), .b(new_n120), .c(new_n108), .d(new_n107), .o1(new_n121));
  nona23aa1n03x5               g026(.a(new_n118), .b(new_n116), .c(new_n121), .d(new_n113), .out0(new_n122));
  inv000aa1n02x5               g027(.a(new_n111), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n112), .o1(new_n124));
  inv000aa1n02x5               g029(.a(new_n115), .o1(new_n125));
  oai022aa1n03x5               g030(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n126));
  nanp03aa1n03x5               g031(.a(new_n126), .b(new_n114), .c(new_n120), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n124), .b(new_n123), .c(new_n127), .d(new_n125), .o1(new_n128));
  inv030aa1n02x5               g033(.a(new_n128), .o1(new_n129));
  oai112aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n122), .d(new_n110), .o1(new_n130));
  nor042aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n97), .out0(\s[10] ));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  inv040aa1n02x5               g040(.a(new_n135), .o1(new_n136));
  tech160nm_finand02aa1n03p5x5 g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoai13aa1n06x5               g042(.a(new_n132), .b(new_n131), .c(new_n130), .d(new_n97), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  nanb02aa1n09x5               g044(.a(new_n135), .b(new_n137), .out0(new_n140));
  nor002aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n09x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  oaoi13aa1n02x7               g048(.a(new_n143), .b(new_n136), .c(new_n138), .d(new_n140), .o1(new_n144));
  oai112aa1n02x7               g049(.a(new_n136), .b(new_n143), .c(new_n138), .d(new_n140), .o1(new_n145));
  norb02aa1n03x4               g050(.a(new_n145), .b(new_n144), .out0(\s[12] ));
  norb02aa1n06x5               g051(.a(new_n105), .b(new_n104), .out0(new_n147));
  oai112aa1n06x5               g052(.a(new_n147), .b(new_n101), .c(new_n102), .d(new_n100), .o1(new_n148));
  nand02aa1n03x5               g053(.a(new_n148), .b(new_n109), .o1(new_n149));
  nona23aa1n03x5               g054(.a(new_n114), .b(new_n111), .c(new_n115), .d(new_n112), .out0(new_n150));
  nor043aa1n03x5               g055(.a(new_n150), .b(new_n117), .c(new_n121), .o1(new_n151));
  nano23aa1n03x5               g056(.a(new_n143), .b(new_n140), .c(new_n133), .d(new_n99), .out0(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n128), .c(new_n151), .d(new_n149), .o1(new_n153));
  oaoi03aa1n02x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n154));
  oai022aa1n02x7               g059(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n155));
  nano23aa1n09x5               g060(.a(new_n143), .b(new_n140), .c(new_n155), .d(new_n132), .out0(new_n156));
  nor022aa1n04x5               g061(.a(new_n156), .b(new_n154), .o1(new_n157));
  nor002aa1n16x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  xobna2aa1n03x5               g065(.a(new_n160), .b(new_n153), .c(new_n157), .out0(\s[13] ));
  inv000aa1d42x5               g066(.a(new_n158), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n153), .d(new_n157), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nand02aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nor042aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand02aa1d04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nor002aa1n03x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(new_n169), .b(new_n158), .o1(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n160), .c(new_n153), .d(new_n157), .o1(new_n171));
  xobna2aa1n03x5               g076(.a(new_n168), .b(new_n171), .c(new_n165), .out0(\s[15] ));
  nanp03aa1n03x5               g077(.a(new_n171), .b(new_n165), .c(new_n168), .o1(new_n173));
  nor042aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand22aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  oaoi13aa1n02x5               g082(.a(new_n177), .b(new_n173), .c(\a[15] ), .d(\b[14] ), .o1(new_n178));
  aoi113aa1n02x5               g083(.a(new_n166), .b(new_n176), .c(new_n171), .d(new_n168), .e(new_n165), .o1(new_n179));
  nor002aa1n02x5               g084(.a(new_n178), .b(new_n179), .o1(\s[16] ));
  nano23aa1n06x5               g085(.a(new_n135), .b(new_n141), .c(new_n142), .d(new_n137), .out0(new_n181));
  nano23aa1n03x7               g086(.a(new_n131), .b(new_n98), .c(new_n132), .d(new_n97), .out0(new_n182));
  nano23aa1n06x5               g087(.a(new_n166), .b(new_n174), .c(new_n175), .d(new_n167), .out0(new_n183));
  nano23aa1n06x5               g088(.a(new_n158), .b(new_n169), .c(new_n165), .d(new_n159), .out0(new_n184));
  nand22aa1n03x5               g089(.a(new_n184), .b(new_n183), .o1(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n181), .c(new_n182), .out0(new_n186));
  aoai13aa1n09x5               g091(.a(new_n186), .b(new_n128), .c(new_n149), .d(new_n151), .o1(new_n187));
  nona23aa1n09x5               g092(.a(new_n175), .b(new_n167), .c(new_n166), .d(new_n174), .out0(new_n188));
  norb02aa1n03x5               g093(.a(new_n184), .b(new_n188), .out0(new_n189));
  tech160nm_fioai012aa1n03p5x5 g094(.a(new_n175), .b(new_n174), .c(new_n166), .o1(new_n190));
  oai012aa1n02x5               g095(.a(new_n165), .b(new_n169), .c(new_n158), .o1(new_n191));
  tech160nm_fioai012aa1n04x5   g096(.a(new_n190), .b(new_n188), .c(new_n191), .o1(new_n192));
  oaoi13aa1n12x5               g097(.a(new_n192), .b(new_n189), .c(new_n156), .d(new_n154), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n187), .c(new_n193), .out0(\s[17] ));
  nor042aa1d18x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  inv000aa1n02x5               g101(.a(new_n196), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n194), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n187), .d(new_n193), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g105(.a(\a[18] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[17] ), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand22aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoi012aa1n02x5               g110(.a(new_n196), .b(new_n201), .c(new_n202), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n198), .c(new_n187), .d(new_n193), .o1(new_n207));
  oai112aa1n06x5               g112(.a(new_n207), .b(new_n205), .c(new_n202), .d(new_n201), .o1(new_n208));
  oaoi13aa1n02x5               g113(.a(new_n205), .b(new_n207), .c(new_n201), .d(new_n202), .o1(new_n209));
  norb02aa1n03x4               g114(.a(new_n208), .b(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n02x5               g116(.a(new_n203), .o1(new_n212));
  nor022aa1n04x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand22aa1n04x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aobi12aa1n06x5               g120(.a(new_n215), .b(new_n208), .c(new_n212), .out0(new_n216));
  nona22aa1n02x5               g121(.a(new_n208), .b(new_n215), .c(new_n203), .out0(new_n217));
  norb02aa1n03x4               g122(.a(new_n217), .b(new_n216), .out0(\s[20] ));
  xorc02aa1n02x5               g123(.a(\a[18] ), .b(\b[17] ), .out0(new_n219));
  nano23aa1n06x5               g124(.a(new_n203), .b(new_n213), .c(new_n214), .d(new_n204), .out0(new_n220));
  nand23aa1n03x5               g125(.a(new_n220), .b(new_n194), .c(new_n219), .o1(new_n221));
  nona23aa1n09x5               g126(.a(new_n214), .b(new_n204), .c(new_n203), .d(new_n213), .out0(new_n222));
  oaoi03aa1n03x5               g127(.a(new_n201), .b(new_n202), .c(new_n196), .o1(new_n223));
  oaoi03aa1n12x5               g128(.a(\a[20] ), .b(\b[19] ), .c(new_n212), .o1(new_n224));
  oabi12aa1n18x5               g129(.a(new_n224), .b(new_n222), .c(new_n223), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n221), .c(new_n187), .d(new_n193), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[20] ), .b(\a[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n229), .c(new_n227), .d(new_n231), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n229), .b(new_n233), .c(new_n227), .d(new_n231), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(\s[22] ));
  nand22aa1n03x5               g141(.a(new_n187), .b(new_n193), .o1(new_n237));
  nona32aa1n02x4               g142(.a(new_n237), .b(new_n232), .c(new_n230), .d(new_n221), .out0(new_n238));
  tech160nm_fioaoi03aa1n05x5   g143(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n239));
  nor042aa1n06x5               g144(.a(new_n232), .b(new_n230), .o1(new_n240));
  aoai13aa1n12x5               g145(.a(new_n240), .b(new_n224), .c(new_n220), .d(new_n239), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(new_n242), .b(new_n243), .c(new_n229), .o1(new_n244));
  nand02aa1d08x5               g149(.a(new_n241), .b(new_n244), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  xnrc02aa1n12x5               g151(.a(\b[22] ), .b(\a[23] ), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xnbna2aa1n03x5               g153(.a(new_n248), .b(new_n238), .c(new_n246), .out0(\s[23] ));
  nor042aa1n06x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n240), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n252), .b(new_n221), .c(new_n187), .d(new_n193), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n248), .b(new_n253), .c(new_n245), .o1(new_n254));
  tech160nm_fixnrc02aa1n02p5x5 g159(.a(\b[23] ), .b(\a[24] ), .out0(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n254), .c(new_n251), .o1(new_n256));
  tech160nm_fiaoi012aa1n05x5   g161(.a(new_n247), .b(new_n238), .c(new_n246), .o1(new_n257));
  nano22aa1n02x4               g162(.a(new_n257), .b(new_n251), .c(new_n255), .out0(new_n258));
  norp02aa1n02x5               g163(.a(new_n256), .b(new_n258), .o1(\s[24] ));
  nor042aa1n02x5               g164(.a(new_n255), .b(new_n247), .o1(new_n260));
  inv030aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  nona32aa1n02x4               g166(.a(new_n237), .b(new_n261), .c(new_n252), .d(new_n221), .out0(new_n262));
  oao003aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n263));
  aoai13aa1n12x5               g168(.a(new_n263), .b(new_n261), .c(new_n241), .d(new_n244), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n262), .c(new_n265), .out0(\s[25] ));
  nor042aa1n03x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n260), .b(new_n240), .o1(new_n270));
  aoi112aa1n02x7               g175(.a(new_n270), .b(new_n221), .c(new_n187), .d(new_n193), .o1(new_n271));
  oai012aa1n02x5               g176(.a(new_n266), .b(new_n271), .c(new_n264), .o1(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  aoi012aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n269), .o1(new_n274));
  aobi12aa1n06x5               g179(.a(new_n266), .b(new_n262), .c(new_n265), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n275), .b(new_n269), .c(new_n273), .out0(new_n276));
  norp02aa1n02x5               g181(.a(new_n274), .b(new_n276), .o1(\s[26] ));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  norb02aa1n03x5               g184(.a(new_n266), .b(new_n273), .out0(new_n280));
  nand02aa1d10x5               g185(.a(new_n264), .b(new_n280), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n282));
  tech160nm_fioai012aa1n03p5x5 g187(.a(new_n129), .b(new_n122), .c(new_n110), .o1(new_n283));
  oabi12aa1n02x5               g188(.a(new_n192), .b(new_n157), .c(new_n185), .out0(new_n284));
  norb03aa1n02x5               g189(.a(new_n280), .b(new_n270), .c(new_n221), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .d(new_n186), .o1(new_n286));
  aoi013aa1n06x4               g191(.a(new_n279), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n287));
  nona23aa1n03x5               g192(.a(new_n280), .b(new_n260), .c(new_n221), .d(new_n252), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n282), .b(new_n288), .c(new_n187), .d(new_n193), .o1(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n281), .c(new_n279), .out0(new_n290));
  nor002aa1n02x5               g195(.a(new_n287), .b(new_n290), .o1(\s[27] ));
  nor042aa1n03x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n244), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n260), .b(new_n294), .c(new_n225), .d(new_n240), .o1(new_n295));
  aobi12aa1n06x5               g200(.a(new_n280), .b(new_n295), .c(new_n263), .out0(new_n296));
  oai012aa1n03x5               g201(.a(new_n278), .b(new_n289), .c(new_n296), .o1(new_n297));
  xnrc02aa1n12x5               g202(.a(\b[27] ), .b(\a[28] ), .out0(new_n298));
  aoi012aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n293), .o1(new_n299));
  nano22aa1n03x5               g204(.a(new_n287), .b(new_n293), .c(new_n298), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n299), .b(new_n300), .o1(\s[28] ));
  xnrc02aa1n02x5               g206(.a(\b[28] ), .b(\a[29] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n278), .b(new_n298), .out0(new_n303));
  tech160nm_fioai012aa1n05x5   g208(.a(new_n303), .b(new_n289), .c(new_n296), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n305));
  aoi012aa1n02x5               g210(.a(new_n302), .b(new_n304), .c(new_n305), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n303), .o1(new_n307));
  aoi013aa1n03x5               g212(.a(new_n307), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n302), .c(new_n305), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n306), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .out0(new_n312));
  norb03aa1n12x5               g217(.a(new_n278), .b(new_n302), .c(new_n298), .out0(new_n313));
  tech160nm_fioai012aa1n02p5x5 g218(.a(new_n313), .b(new_n289), .c(new_n296), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n315));
  aoi012aa1n02x5               g220(.a(new_n312), .b(new_n314), .c(new_n315), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n313), .o1(new_n317));
  aoi013aa1n02x5               g222(.a(new_n317), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n312), .c(new_n315), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n316), .b(new_n319), .o1(\s[30] ));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  norb02aa1n02x7               g226(.a(new_n313), .b(new_n312), .out0(new_n322));
  inv000aa1n02x5               g227(.a(new_n322), .o1(new_n323));
  aoi013aa1n02x5               g228(.a(new_n323), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n325));
  nano22aa1n03x5               g230(.a(new_n324), .b(new_n321), .c(new_n325), .out0(new_n326));
  tech160nm_fioai012aa1n02p5x5 g231(.a(new_n322), .b(new_n289), .c(new_n296), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n321), .b(new_n327), .c(new_n325), .o1(new_n328));
  norp02aa1n03x5               g233(.a(new_n328), .b(new_n326), .o1(\s[31] ));
  xobna2aa1n03x5               g234(.a(new_n147), .b(new_n103), .c(new_n101), .out0(\s[3] ));
  aoi012aa1n02x5               g235(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[3] ), .c(new_n107), .out0(\s[4] ));
  aoi022aa1n02x5               g237(.a(new_n148), .b(new_n109), .c(\b[3] ), .d(\a[4] ), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  xorc02aa1n02x5               g239(.a(\a[6] ), .b(\b[5] ), .out0(new_n335));
  oai112aa1n02x5               g240(.a(new_n149), .b(new_n118), .c(new_n108), .d(new_n107), .o1(new_n336));
  oaoi13aa1n02x5               g241(.a(new_n335), .b(new_n336), .c(\a[5] ), .d(\b[4] ), .o1(new_n337));
  oai112aa1n02x5               g242(.a(new_n336), .b(new_n335), .c(\b[4] ), .d(\a[5] ), .o1(new_n338));
  nanb02aa1n02x5               g243(.a(new_n337), .b(new_n338), .out0(\s[6] ));
  xobna2aa1n03x5               g244(.a(new_n116), .b(new_n338), .c(new_n120), .out0(\s[7] ));
  nanp03aa1n02x5               g245(.a(new_n338), .b(new_n116), .c(new_n120), .o1(new_n341));
  xobna2aa1n03x5               g246(.a(new_n113), .b(new_n341), .c(new_n125), .out0(\s[8] ));
  xorb03aa1n02x5               g247(.a(new_n283), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

