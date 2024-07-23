// Benchmark "adder" written by ABC on Thu Jul 18 07:43:07 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n333,
    new_n335, new_n336, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n09x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  inv000aa1n02x5               g007(.a(new_n102), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano23aa1n02x5               g012(.a(new_n106), .b(new_n105), .c(new_n107), .d(new_n104), .out0(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n109));
  aobi12aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .out0(new_n110));
  nanp02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nor022aa1n03x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nona23aa1n03x5               g019(.a(new_n113), .b(new_n111), .c(new_n114), .d(new_n112), .out0(new_n115));
  tech160nm_finand02aa1n03p5x5 g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nor002aa1d32x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor022aa1n16x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1d15x5               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  nanb02aa1n09x5               g025(.a(new_n115), .b(new_n120), .out0(new_n121));
  oai012aa1n02x5               g026(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[4] ), .o1(new_n123));
  nanb02aa1n03x5               g028(.a(\a[5] ), .b(new_n123), .out0(new_n124));
  oaoi03aa1n09x5               g029(.a(\a[6] ), .b(\b[5] ), .c(new_n124), .o1(new_n125));
  aobi12aa1n12x5               g030(.a(new_n122), .b(new_n120), .c(new_n125), .out0(new_n126));
  oai012aa1n09x5               g031(.a(new_n126), .b(new_n110), .c(new_n121), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  nand02aa1d12x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  inv030aa1n04x5               g037(.a(new_n132), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor002aa1n16x5               g039(.a(\b[8] ), .b(\a[9] ), .o1(new_n135));
  and002aa1n12x5               g040(.a(\b[8] ), .b(\a[9] ), .o(new_n136));
  nona22aa1n02x4               g041(.a(new_n127), .b(new_n136), .c(new_n135), .out0(new_n137));
  nor002aa1d24x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  nona22aa1n06x5               g043(.a(new_n137), .b(new_n138), .c(new_n135), .out0(new_n139));
  nona32aa1n06x5               g044(.a(new_n139), .b(new_n134), .c(new_n133), .d(new_n131), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n134), .o1(new_n141));
  aoi022aa1n02x5               g046(.a(new_n139), .b(new_n130), .c(new_n132), .d(new_n141), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n140), .b(new_n142), .out0(\s[11] ));
  norp02aa1n24x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1n20x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aoi113aa1n02x5               g051(.a(new_n134), .b(new_n146), .c(new_n139), .d(new_n130), .e(new_n132), .o1(new_n147));
  aobi12aa1n06x5               g052(.a(new_n146), .b(new_n140), .c(new_n141), .out0(new_n148));
  norp02aa1n02x5               g053(.a(new_n148), .b(new_n147), .o1(\s[12] ));
  nona23aa1n02x5               g054(.a(new_n104), .b(new_n107), .c(new_n106), .d(new_n105), .out0(new_n150));
  oaih12aa1n06x5               g055(.a(new_n109), .b(new_n150), .c(new_n102), .o1(new_n151));
  nona23aa1n06x5               g056(.a(new_n118), .b(new_n116), .c(new_n119), .d(new_n117), .out0(new_n152));
  nor002aa1n03x5               g057(.a(new_n152), .b(new_n115), .o1(new_n153));
  oaib12aa1n09x5               g058(.a(new_n122), .b(new_n152), .c(new_n125), .out0(new_n154));
  norb03aa1d15x5               g059(.a(new_n130), .b(new_n135), .c(new_n138), .out0(new_n155));
  norb03aa1d15x5               g060(.a(new_n145), .b(new_n134), .c(new_n144), .out0(new_n156));
  nona23aa1d16x5               g061(.a(new_n155), .b(new_n156), .c(new_n133), .d(new_n136), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n154), .c(new_n151), .d(new_n153), .o1(new_n159));
  nona23aa1d18x5               g064(.a(new_n132), .b(new_n145), .c(new_n144), .d(new_n134), .out0(new_n160));
  tech160nm_fioai012aa1n03p5x5 g065(.a(new_n145), .b(new_n144), .c(new_n134), .o1(new_n161));
  aoai13aa1n12x5               g066(.a(new_n130), .b(new_n138), .c(new_n97), .d(new_n98), .o1(new_n162));
  oai012aa1d24x5               g067(.a(new_n161), .b(new_n160), .c(new_n162), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nor002aa1d24x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n03x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n159), .c(new_n164), .out0(\s[13] ));
  inv000aa1d42x5               g073(.a(new_n166), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n167), .b(new_n163), .c(new_n127), .d(new_n158), .o1(new_n170));
  norp02aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  norb02aa1n03x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nona23aa1n03x5               g079(.a(new_n165), .b(new_n172), .c(new_n171), .d(new_n166), .out0(new_n175));
  oai012aa1n02x7               g080(.a(new_n172), .b(new_n171), .c(new_n166), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n176), .b(new_n175), .c(new_n159), .d(new_n164), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nand02aa1d06x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nor042aa1n06x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nor022aa1n08x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand22aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aoi112aa1n03x5               g088(.a(new_n183), .b(new_n180), .c(new_n177), .d(new_n179), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n183), .b(new_n180), .c(new_n177), .d(new_n179), .o1(new_n185));
  norb02aa1n02x7               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nano23aa1n06x5               g091(.a(new_n181), .b(new_n180), .c(new_n182), .d(new_n179), .out0(new_n187));
  nano32aa1d12x5               g092(.a(new_n157), .b(new_n187), .c(new_n167), .d(new_n173), .out0(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n154), .c(new_n151), .d(new_n153), .o1(new_n189));
  nona23aa1n09x5               g094(.a(new_n179), .b(new_n182), .c(new_n181), .d(new_n180), .out0(new_n190));
  nor042aa1n02x5               g095(.a(new_n190), .b(new_n175), .o1(new_n191));
  nor002aa1n02x5               g096(.a(new_n190), .b(new_n176), .o1(new_n192));
  oa0012aa1n02x5               g097(.a(new_n182), .b(new_n181), .c(new_n180), .o(new_n193));
  aoi112aa1n09x5               g098(.a(new_n193), .b(new_n192), .c(new_n163), .d(new_n191), .o1(new_n194));
  nand02aa1d08x5               g099(.a(new_n189), .b(new_n194), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g101(.a(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[16] ), .o1(new_n198));
  oaoi03aa1n03x5               g103(.a(new_n197), .b(new_n198), .c(new_n195), .o1(new_n199));
  xnrb03aa1n03x5               g104(.a(new_n199), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g105(.a(new_n198), .b(new_n197), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  nor042aa1n06x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand02aa1d08x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nanb02aa1n12x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  nano22aa1d15x5               g110(.a(new_n205), .b(new_n201), .c(new_n202), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n12x5               g112(.a(new_n204), .b(new_n203), .c(new_n197), .d(new_n198), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n207), .c(new_n189), .d(new_n194), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nor042aa1n12x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nor042aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoi112aa1n03x4               g122(.a(new_n217), .b(new_n213), .c(new_n209), .d(new_n212), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n213), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n212), .b(new_n213), .out0(new_n220));
  nanp02aa1n03x5               g125(.a(new_n209), .b(new_n220), .o1(new_n221));
  tech160nm_fiaoi012aa1n02p5x5 g126(.a(new_n216), .b(new_n221), .c(new_n219), .o1(new_n222));
  norp02aa1n03x5               g127(.a(new_n222), .b(new_n218), .o1(\s[20] ));
  nano23aa1n06x5               g128(.a(new_n214), .b(new_n213), .c(new_n215), .d(new_n212), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n206), .b(new_n224), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n208), .o1(new_n226));
  oai012aa1n02x5               g131(.a(new_n215), .b(new_n214), .c(new_n213), .o1(new_n227));
  aobi12aa1n06x5               g132(.a(new_n227), .b(new_n224), .c(new_n226), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n225), .c(new_n189), .d(new_n194), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nanp02aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nor042aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .out0(new_n233));
  aoi112aa1n03x4               g138(.a(new_n233), .b(new_n232), .c(new_n229), .d(new_n231), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n232), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n231), .b(new_n232), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n229), .b(new_n236), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n233), .o1(new_n238));
  tech160nm_fiaoi012aa1n03p5x5 g143(.a(new_n238), .b(new_n237), .c(new_n235), .o1(new_n239));
  nor042aa1n03x5               g144(.a(new_n239), .b(new_n234), .o1(\s[22] ));
  norp02aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nano23aa1n06x5               g147(.a(new_n241), .b(new_n232), .c(new_n242), .d(new_n231), .out0(new_n243));
  nand23aa1n03x5               g148(.a(new_n206), .b(new_n224), .c(new_n243), .o1(new_n244));
  nona23aa1n02x4               g149(.a(new_n212), .b(new_n215), .c(new_n214), .d(new_n213), .out0(new_n245));
  oai012aa1n06x5               g150(.a(new_n227), .b(new_n245), .c(new_n208), .o1(new_n246));
  tech160nm_fioaoi03aa1n03p5x5 g151(.a(\a[22] ), .b(\b[21] ), .c(new_n235), .o1(new_n247));
  aoi012aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n243), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n244), .c(new_n189), .d(new_n194), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n16x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  nor042aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nand02aa1n04x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  aoi112aa1n03x4               g160(.a(new_n251), .b(new_n255), .c(new_n249), .d(new_n252), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n251), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n252), .b(new_n251), .out0(new_n258));
  nanp02aa1n03x5               g163(.a(new_n249), .b(new_n258), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n255), .o1(new_n260));
  tech160nm_fiaoi012aa1n03p5x5 g165(.a(new_n260), .b(new_n259), .c(new_n257), .o1(new_n261));
  nor042aa1n03x5               g166(.a(new_n261), .b(new_n256), .o1(\s[24] ));
  nano23aa1n06x5               g167(.a(new_n251), .b(new_n253), .c(new_n254), .d(new_n252), .out0(new_n263));
  nanb03aa1n02x5               g168(.a(new_n225), .b(new_n263), .c(new_n243), .out0(new_n264));
  nona22aa1n02x4               g169(.a(new_n254), .b(new_n253), .c(new_n251), .out0(new_n265));
  aoi022aa1n06x5               g170(.a(new_n263), .b(new_n247), .c(new_n265), .d(new_n254), .o1(new_n266));
  inv020aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aoi013aa1n02x4               g172(.a(new_n267), .b(new_n246), .c(new_n243), .d(new_n263), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n264), .c(new_n189), .d(new_n194), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nanp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  nor042aa1n04x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  xnrc02aa1n12x5               g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoi112aa1n02x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n271), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n272), .o1(new_n276));
  norb02aa1n02x5               g181(.a(new_n271), .b(new_n272), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n269), .b(new_n277), .o1(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n273), .b(new_n278), .c(new_n276), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n279), .b(new_n275), .o1(\s[26] ));
  nanp02aa1n02x5               g185(.a(new_n163), .b(new_n191), .o1(new_n281));
  nona22aa1n02x4               g186(.a(new_n281), .b(new_n193), .c(new_n192), .out0(new_n282));
  nano22aa1n12x5               g187(.a(new_n273), .b(new_n271), .c(new_n276), .out0(new_n283));
  nano22aa1n03x7               g188(.a(new_n244), .b(new_n263), .c(new_n283), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n282), .c(new_n127), .d(new_n188), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n228), .b(new_n243), .c(new_n263), .out0(new_n286));
  oaoi03aa1n02x5               g191(.a(\a[26] ), .b(\b[25] ), .c(new_n276), .o1(new_n287));
  oaoi13aa1n09x5               g192(.a(new_n287), .b(new_n283), .c(new_n286), .d(new_n267), .o1(new_n288));
  nor042aa1n03x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norb02aa1n02x5               g195(.a(new_n290), .b(new_n289), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n285), .c(new_n288), .out0(\s[27] ));
  inv000aa1d42x5               g197(.a(new_n289), .o1(new_n293));
  aobi12aa1n02x7               g198(.a(new_n291), .b(new_n285), .c(new_n288), .out0(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[27] ), .b(\a[28] ), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n294), .b(new_n293), .c(new_n295), .out0(new_n296));
  nanp03aa1n02x5               g201(.a(new_n246), .b(new_n243), .c(new_n263), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n283), .o1(new_n298));
  inv000aa1n02x5               g203(.a(new_n287), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n297), .d(new_n266), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n291), .b(new_n300), .c(new_n195), .d(new_n284), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n295), .b(new_n301), .c(new_n293), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n296), .o1(\s[28] ));
  xnrc02aa1n02x5               g208(.a(\b[28] ), .b(\a[29] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n295), .b(new_n293), .c(new_n290), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n300), .c(new_n195), .d(new_n284), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n304), .b(new_n306), .c(new_n307), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n305), .b(new_n285), .c(new_n288), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n304), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g217(.a(new_n304), .b(new_n295), .c(new_n290), .d(new_n293), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n300), .c(new_n195), .d(new_n284), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n06x5               g222(.a(new_n313), .b(new_n285), .c(new_n288), .out0(new_n318));
  nano22aa1n02x4               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  nor002aa1n02x5               g224(.a(new_n317), .b(new_n319), .o1(\s[30] ));
  norb03aa1n02x5               g225(.a(new_n305), .b(new_n304), .c(new_n316), .out0(new_n321));
  aobi12aa1n02x7               g226(.a(new_n321), .b(new_n285), .c(new_n288), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[30] ), .b(\a[31] ), .out0(new_n324));
  nano22aa1n02x4               g229(.a(new_n322), .b(new_n323), .c(new_n324), .out0(new_n325));
  aoai13aa1n03x5               g230(.a(new_n321), .b(new_n300), .c(new_n195), .d(new_n284), .o1(new_n326));
  tech160nm_fiaoi012aa1n02p5x5 g231(.a(new_n324), .b(new_n326), .c(new_n323), .o1(new_n327));
  norp02aa1n03x5               g232(.a(new_n327), .b(new_n325), .o1(\s[31] ));
  xnrb03aa1n02x5               g233(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g234(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g236(.a(new_n151), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g237(.a(\a[5] ), .b(\b[4] ), .c(new_n110), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g239(.a(new_n116), .b(new_n119), .out0(new_n335));
  oai112aa1n02x5               g240(.a(new_n113), .b(new_n335), .c(new_n333), .d(new_n114), .o1(new_n336));
  oaoi13aa1n02x5               g241(.a(new_n335), .b(new_n113), .c(new_n333), .d(new_n114), .o1(new_n337));
  norb02aa1n02x5               g242(.a(new_n336), .b(new_n337), .out0(\s[7] ));
  oai012aa1n02x5               g243(.a(new_n336), .b(\b[6] ), .c(\a[7] ), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


