// Benchmark "adder" written by ABC on Thu Jul 18 02:10:32 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n334, new_n337, new_n339, new_n341, new_n342,
    new_n343, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  oai112aa1n04x5               g007(.a(new_n102), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n103));
  oa0022aa1n09x5               g008(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n104));
  aoi022aa1n06x5               g009(.a(new_n103), .b(new_n104), .c(\b[3] ), .d(\a[4] ), .o1(new_n105));
  nor002aa1n04x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nanp02aa1n03x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .out0(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .out0(new_n112));
  nor043aa1n03x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[6] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[5] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  oaoi03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n108), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n119));
  oabi12aa1n06x5               g024(.a(new_n119), .b(new_n110), .c(new_n117), .out0(new_n120));
  nand42aa1n02x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  norb02aa1n02x5               g026(.a(new_n121), .b(new_n97), .out0(new_n122));
  aoai13aa1n03x5               g027(.a(new_n122), .b(new_n120), .c(new_n113), .d(new_n105), .o1(new_n123));
  nor042aa1n09x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(new_n124), .o1(new_n128));
  inv000aa1n02x5               g033(.a(new_n126), .o1(new_n129));
  aoai13aa1n04x5               g034(.a(new_n128), .b(new_n129), .c(new_n123), .d(new_n98), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nor002aa1d32x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n09x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  aoai13aa1n03x5               g043(.a(new_n138), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n130), .b(new_n134), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n138), .c(new_n132), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n141), .b(new_n139), .o1(\s[12] ));
  nano23aa1d15x5               g047(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n143));
  nano23aa1n06x5               g048(.a(new_n97), .b(new_n124), .c(new_n125), .d(new_n121), .out0(new_n144));
  nand22aa1n09x5               g049(.a(new_n144), .b(new_n143), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n03x5               g051(.a(new_n146), .b(new_n120), .c(new_n113), .d(new_n105), .o1(new_n147));
  aoi012aa1n06x5               g052(.a(new_n124), .b(new_n97), .c(new_n125), .o1(new_n148));
  inv040aa1n03x5               g053(.a(new_n148), .o1(new_n149));
  oaih12aa1n02x5               g054(.a(new_n136), .b(new_n135), .c(new_n132), .o1(new_n150));
  aobi12aa1n12x5               g055(.a(new_n150), .b(new_n143), .c(new_n149), .out0(new_n151));
  nor042aa1n04x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n03x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n147), .c(new_n151), .out0(\s[13] ));
  orn002aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .o(new_n156));
  nanp02aa1n02x5               g061(.a(\b[3] ), .b(\a[4] ), .o1(new_n157));
  nanp02aa1n03x5               g062(.a(new_n103), .b(new_n104), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  nano23aa1n02x5               g064(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n160));
  xorc02aa1n02x5               g065(.a(\a[6] ), .b(\b[5] ), .out0(new_n161));
  xorc02aa1n02x5               g066(.a(\a[5] ), .b(\b[4] ), .out0(new_n162));
  nanp03aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n162), .o1(new_n163));
  oab012aa1n06x5               g068(.a(new_n119), .b(new_n110), .c(new_n117), .out0(new_n164));
  tech160nm_fioai012aa1n02p5x5 g069(.a(new_n164), .b(new_n159), .c(new_n163), .o1(new_n165));
  nona23aa1n03x5               g070(.a(new_n136), .b(new_n133), .c(new_n132), .d(new_n135), .out0(new_n166));
  tech160nm_fioai012aa1n05x5   g071(.a(new_n150), .b(new_n166), .c(new_n148), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n154), .b(new_n167), .c(new_n165), .d(new_n146), .o1(new_n168));
  nor042aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n06x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x7               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n168), .c(new_n156), .out0(\s[14] ));
  nona23aa1n09x5               g077(.a(new_n170), .b(new_n153), .c(new_n152), .d(new_n169), .out0(new_n173));
  aoi012aa1n02x5               g078(.a(new_n169), .b(new_n152), .c(new_n170), .o1(new_n174));
  aoai13aa1n04x5               g079(.a(new_n174), .b(new_n173), .c(new_n147), .d(new_n151), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanp02aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nor042aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n04x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  aoai13aa1n03x5               g086(.a(new_n181), .b(new_n177), .c(new_n175), .d(new_n178), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n178), .b(new_n177), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n175), .b(new_n183), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n181), .c(new_n177), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n182), .o1(\s[16] ));
  nano23aa1n03x7               g091(.a(new_n177), .b(new_n179), .c(new_n180), .d(new_n178), .out0(new_n187));
  nano32aa1n03x7               g092(.a(new_n145), .b(new_n187), .c(new_n154), .d(new_n171), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n120), .c(new_n105), .d(new_n113), .o1(new_n189));
  nona23aa1n03x5               g094(.a(new_n180), .b(new_n178), .c(new_n177), .d(new_n179), .out0(new_n190));
  norp02aa1n06x5               g095(.a(new_n190), .b(new_n173), .o1(new_n191));
  aoai13aa1n04x5               g096(.a(new_n178), .b(new_n169), .c(new_n152), .d(new_n170), .o1(new_n192));
  nona22aa1n03x5               g097(.a(new_n192), .b(new_n179), .c(new_n177), .out0(new_n193));
  aoi022aa1d18x5               g098(.a(new_n167), .b(new_n191), .c(new_n180), .d(new_n193), .o1(new_n194));
  xnrc02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n194), .out0(\s[17] ));
  nand22aa1n03x5               g101(.a(new_n113), .b(new_n105), .o1(new_n197));
  nona23aa1n03x5               g102(.a(new_n187), .b(new_n144), .c(new_n166), .d(new_n173), .out0(new_n198));
  aoai13aa1n09x5               g103(.a(new_n194), .b(new_n198), .c(new_n197), .d(new_n164), .o1(new_n199));
  nor042aa1n12x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  nand42aa1n20x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  nor042aa1n09x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand42aa1d28x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n200), .c(new_n199), .d(new_n201), .o1(new_n205));
  aoi112aa1n02x7               g110(.a(new_n200), .b(new_n204), .c(new_n199), .d(new_n201), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n205), .b(new_n206), .out0(\s[18] ));
  nano23aa1d15x5               g112(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoi012aa1d24x5               g114(.a(new_n202), .b(new_n200), .c(new_n203), .o1(new_n210));
  aoai13aa1n04x5               g115(.a(new_n210), .b(new_n209), .c(new_n189), .d(new_n194), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n04x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand42aa1n03x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  xnrc02aa1n12x5               g120(.a(\b[19] ), .b(\a[20] ), .out0(new_n216));
  aoai13aa1n03x5               g121(.a(new_n216), .b(new_n214), .c(new_n211), .d(new_n215), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n210), .o1(new_n218));
  nanb02aa1n06x5               g123(.a(new_n214), .b(new_n215), .out0(new_n219));
  inv000aa1n02x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n220), .b(new_n218), .c(new_n199), .d(new_n208), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n216), .c(new_n214), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n217), .b(new_n222), .o1(\s[20] ));
  nano22aa1n12x5               g128(.a(new_n216), .b(new_n208), .c(new_n220), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[19] ), .o1(new_n226));
  aoai13aa1n04x5               g131(.a(new_n215), .b(new_n202), .c(new_n200), .d(new_n203), .o1(new_n227));
  oab012aa1n02x4               g132(.a(new_n214), .b(\a[20] ), .c(\b[19] ), .out0(new_n228));
  nanp02aa1n09x5               g133(.a(new_n227), .b(new_n228), .o1(new_n229));
  oaib12aa1n18x5               g134(.a(new_n229), .b(new_n226), .c(\a[20] ), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n225), .c(new_n189), .d(new_n194), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n06x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nanp02aa1n04x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  nor022aa1n04x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  nand22aa1n04x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n231), .d(new_n235), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n230), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n235), .b(new_n240), .c(new_n199), .d(new_n224), .o1(new_n241));
  nona22aa1n03x5               g146(.a(new_n241), .b(new_n238), .c(new_n233), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n239), .b(new_n242), .o1(\s[22] ));
  nona23aa1d18x5               g148(.a(new_n237), .b(new_n234), .c(new_n233), .d(new_n236), .out0(new_n244));
  inv040aa1n04x5               g149(.a(new_n244), .o1(new_n245));
  nona23aa1d16x5               g150(.a(new_n245), .b(new_n208), .c(new_n219), .d(new_n216), .out0(new_n246));
  tech160nm_fiaoi012aa1n04x5   g151(.a(new_n236), .b(new_n233), .c(new_n237), .o1(new_n247));
  oa0012aa1n12x5               g152(.a(new_n247), .b(new_n230), .c(new_n244), .o(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n246), .c(new_n189), .d(new_n194), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[22] ), .b(\a[23] ), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  tech160nm_fixnrc02aa1n04x5   g158(.a(\b[23] ), .b(\a[24] ), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n251), .c(new_n249), .d(new_n253), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n246), .o1(new_n256));
  inv000aa1n03x5               g161(.a(new_n248), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n253), .b(new_n257), .c(new_n199), .d(new_n256), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n258), .b(new_n254), .c(new_n251), .out0(new_n259));
  nanp02aa1n03x5               g164(.a(new_n255), .b(new_n259), .o1(\s[24] ));
  nona32aa1n06x5               g165(.a(new_n224), .b(new_n254), .c(new_n252), .d(new_n244), .out0(new_n261));
  nor042aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aob012aa1n02x5               g168(.a(new_n251), .b(\b[23] ), .c(\a[24] ), .out0(new_n264));
  norp03aa1n06x5               g169(.a(new_n254), .b(new_n252), .c(new_n247), .o1(new_n265));
  nano22aa1n03x7               g170(.a(new_n265), .b(new_n263), .c(new_n264), .out0(new_n266));
  and002aa1n02x5               g171(.a(\b[19] ), .b(\a[20] ), .o(new_n267));
  nor042aa1n06x5               g172(.a(new_n254), .b(new_n252), .o1(new_n268));
  nona23aa1n08x5               g173(.a(new_n229), .b(new_n268), .c(new_n244), .d(new_n267), .out0(new_n269));
  nand22aa1n12x5               g174(.a(new_n269), .b(new_n266), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n04x5               g176(.a(new_n271), .b(new_n261), .c(new_n189), .d(new_n194), .o1(new_n272));
  xorb03aa1n02x5               g177(.a(new_n272), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  tech160nm_fixorc02aa1n05x5   g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  tech160nm_fixnrc02aa1n04x5   g180(.a(\b[25] ), .b(\a[26] ), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n275), .o1(new_n277));
  inv030aa1n02x5               g182(.a(new_n261), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n275), .b(new_n270), .c(new_n199), .d(new_n278), .o1(new_n279));
  nona22aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n274), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n277), .b(new_n280), .o1(\s[26] ));
  nanp02aa1n02x5               g186(.a(new_n193), .b(new_n180), .o1(new_n282));
  oaib12aa1n06x5               g187(.a(new_n282), .b(new_n151), .c(new_n191), .out0(new_n283));
  norb02aa1n12x5               g188(.a(new_n275), .b(new_n276), .out0(new_n284));
  nano22aa1d15x5               g189(.a(new_n246), .b(new_n268), .c(new_n284), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n283), .c(new_n165), .d(new_n188), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(\b[25] ), .b(\a[26] ), .o1(new_n287));
  oai022aa1n02x5               g192(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n288));
  aoi022aa1d18x5               g193(.a(new_n270), .b(new_n284), .c(new_n287), .d(new_n288), .o1(new_n289));
  tech160nm_fixnrc02aa1n04x5   g194(.a(\b[26] ), .b(\a[27] ), .out0(new_n290));
  xobna2aa1n06x5               g195(.a(new_n290), .b(new_n286), .c(new_n289), .out0(\s[27] ));
  inv000aa1d42x5               g196(.a(new_n284), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(new_n288), .b(new_n287), .o1(new_n293));
  aoai13aa1n04x5               g198(.a(new_n293), .b(new_n292), .c(new_n269), .d(new_n266), .o1(new_n294));
  and002aa1n02x5               g199(.a(\b[26] ), .b(\a[27] ), .o(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n199), .d(new_n285), .o1(new_n297));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  inv000aa1n03x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n286), .d(new_n289), .o1(new_n300));
  tech160nm_fixorc02aa1n04x5   g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n298), .o1(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n297), .d(new_n302), .o1(\s[28] ));
  norb02aa1n02x7               g208(.a(new_n301), .b(new_n290), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n294), .c(new_n199), .d(new_n285), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n304), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n307));
  aoai13aa1n02x7               g212(.a(new_n307), .b(new_n306), .c(new_n286), .d(new_n289), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n307), .b(new_n309), .out0(new_n310));
  aoi022aa1n02x7               g215(.a(new_n308), .b(new_n309), .c(new_n305), .d(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n12x5               g217(.a(new_n290), .b(new_n301), .c(new_n309), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n294), .c(new_n199), .d(new_n285), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n313), .o1(new_n315));
  oaoi03aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n316));
  inv000aa1n03x5               g221(.a(new_n316), .o1(new_n317));
  aoai13aa1n02x7               g222(.a(new_n317), .b(new_n315), .c(new_n286), .d(new_n289), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[30] ), .b(\b[29] ), .out0(new_n319));
  and002aa1n02x5               g224(.a(\b[28] ), .b(\a[29] ), .o(new_n320));
  oabi12aa1n02x5               g225(.a(new_n319), .b(\a[29] ), .c(\b[28] ), .out0(new_n321));
  oab012aa1n02x4               g226(.a(new_n321), .b(new_n307), .c(new_n320), .out0(new_n322));
  aoi022aa1n03x5               g227(.a(new_n318), .b(new_n319), .c(new_n314), .d(new_n322), .o1(\s[30] ));
  nano32aa1n02x4               g228(.a(new_n290), .b(new_n319), .c(new_n301), .d(new_n309), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n294), .c(new_n199), .d(new_n285), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  inv000aa1n02x5               g233(.a(new_n324), .o1(new_n329));
  aoai13aa1n02x7               g234(.a(new_n327), .b(new_n329), .c(new_n286), .d(new_n289), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n325), .d(new_n328), .o1(\s[31] ));
  oai012aa1n02x5               g236(.a(new_n100), .b(new_n101), .c(new_n99), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g238(.a(\a[3] ), .b(\b[2] ), .c(new_n332), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n105), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g241(.a(\a[5] ), .b(\b[4] ), .c(new_n159), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g243(.a(new_n114), .b(new_n115), .c(new_n337), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n339), .b(new_n118), .c(new_n109), .out0(\s[7] ));
  norb02aa1n02x5               g245(.a(new_n107), .b(new_n106), .out0(new_n341));
  nano22aa1n03x7               g246(.a(new_n339), .b(new_n118), .c(new_n109), .out0(new_n342));
  oabi12aa1n02x5               g247(.a(new_n341), .b(new_n342), .c(new_n108), .out0(new_n343));
  nano22aa1n02x4               g248(.a(new_n342), .b(new_n341), .c(new_n118), .out0(new_n344));
  nanb02aa1n03x5               g249(.a(new_n344), .b(new_n343), .out0(\s[8] ));
  xnbna2aa1n03x5               g250(.a(new_n122), .b(new_n197), .c(new_n164), .out0(\s[9] ));
endmodule


