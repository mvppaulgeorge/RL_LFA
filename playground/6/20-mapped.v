// Benchmark "adder" written by ABC on Wed Jul 17 15:09:35 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  orn002aa1n24x5               g004(.a(\a[8] ), .b(\b[7] ), .o(new_n100));
  nanp02aa1n06x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  orn002aa1n02x7               g006(.a(\a[7] ), .b(\b[6] ), .o(new_n102));
  aob012aa1n03x5               g007(.a(new_n99), .b(new_n100), .c(new_n102), .out0(new_n103));
  tech160nm_fixnrc02aa1n04x5   g008(.a(\b[6] ), .b(\a[7] ), .out0(new_n104));
  nor042aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nand22aa1n03x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n09x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  tech160nm_fiaoi012aa1n05x5   g012(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n108));
  oai013aa1d12x5               g013(.a(new_n103), .b(new_n104), .c(new_n101), .d(new_n108), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nand22aa1n03x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  tech160nm_fiaoi012aa1n04x5   g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n114), .b(new_n117), .c(new_n116), .d(new_n115), .out0(new_n118));
  aoi012aa1n02x5               g023(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n119));
  oaih12aa1n06x5               g024(.a(new_n119), .b(new_n118), .c(new_n113), .o1(new_n120));
  oai112aa1n02x5               g025(.a(new_n100), .b(new_n99), .c(\b[6] ), .d(\a[7] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n107), .b(new_n122), .out0(new_n123));
  aoi012aa1n02x5               g028(.a(new_n105), .b(\a[7] ), .c(\b[6] ), .o1(new_n124));
  nano23aa1n06x5               g029(.a(new_n121), .b(new_n123), .c(new_n124), .d(new_n106), .out0(new_n125));
  and002aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o(new_n126));
  norp02aa1n02x5               g031(.a(new_n126), .b(new_n97), .o1(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n128));
  nor022aa1n08x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  and002aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o(new_n130));
  norp02aa1n02x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  inv000aa1n02x5               g037(.a(new_n131), .o1(new_n133));
  oabi12aa1n06x5               g038(.a(new_n130), .b(new_n97), .c(new_n129), .out0(new_n134));
  tech160nm_fioai012aa1n05x5   g039(.a(new_n134), .b(new_n128), .c(new_n133), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n12x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand42aa1n08x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  tech160nm_fiaoi012aa1n05x5   g043(.a(new_n137), .b(new_n135), .c(new_n138), .o1(new_n139));
  orn002aa1n02x5               g044(.a(\a[12] ), .b(\b[11] ), .o(new_n140));
  nand42aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n139), .b(new_n141), .c(new_n140), .out0(\s[12] ));
  nor043aa1n06x5               g047(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n143));
  norp02aa1n12x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano22aa1n03x7               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(new_n145));
  nona23aa1d18x5               g050(.a(new_n145), .b(new_n143), .c(new_n126), .d(new_n137), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n148));
  oaih12aa1n02x5               g053(.a(new_n141), .b(new_n144), .c(new_n137), .o1(new_n149));
  nona23aa1n06x5               g054(.a(new_n141), .b(new_n138), .c(new_n137), .d(new_n144), .out0(new_n150));
  oai012aa1n18x5               g055(.a(new_n149), .b(new_n150), .c(new_n134), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1d28x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n152), .out0(\s[13] ));
  nanp02aa1n02x5               g061(.a(new_n148), .b(new_n152), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n153), .b(new_n157), .c(new_n154), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n09x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand42aa1n16x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  oai012aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n153), .o1(new_n162));
  nano23aa1d15x5               g067(.a(new_n153), .b(new_n160), .c(new_n161), .d(new_n154), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n162), .b(new_n164), .c(new_n148), .d(new_n152), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1d28x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nor042aa1d18x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand02aa1d08x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n12x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n167), .b(new_n172), .c(new_n165), .d(new_n168), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n174));
  norb02aa1n03x4               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  inv000aa1n02x5               g080(.a(new_n167), .o1(new_n176));
  nanb03aa1d18x5               g081(.a(new_n169), .b(new_n170), .c(new_n168), .out0(new_n177));
  inv000aa1n02x5               g082(.a(new_n177), .o1(new_n178));
  nano32aa1n09x5               g083(.a(new_n146), .b(new_n178), .c(new_n163), .d(new_n176), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n109), .c(new_n120), .d(new_n125), .o1(new_n180));
  nona22aa1n02x4               g085(.a(new_n161), .b(new_n160), .c(new_n153), .out0(new_n181));
  nano23aa1n03x7               g086(.a(new_n181), .b(new_n177), .c(new_n176), .d(new_n154), .out0(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(new_n176), .o1(new_n183));
  nano23aa1n06x5               g088(.a(new_n162), .b(new_n171), .c(new_n176), .d(new_n168), .out0(new_n184));
  aoi112aa1n09x5               g089(.a(new_n184), .b(new_n183), .c(new_n151), .d(new_n182), .o1(new_n185));
  nanp02aa1n06x5               g090(.a(new_n180), .b(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  nand42aa1n03x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  and002aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .o(new_n191));
  aoai13aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n180), .d(new_n185), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n04x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  tech160nm_finand02aa1n03p5x5 g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nano23aa1d15x5               g100(.a(new_n194), .b(new_n191), .c(new_n190), .d(new_n195), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  aoai13aa1n06x5               g102(.a(new_n195), .b(new_n194), .c(new_n188), .d(new_n189), .o1(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n197), .c(new_n180), .d(new_n185), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norp02aa1n04x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand42aa1n03x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n06x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoi112aa1n02x7               g112(.a(new_n202), .b(new_n207), .c(new_n199), .d(new_n203), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n203), .o1(new_n209));
  norb02aa1n03x4               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nona23aa1n09x5               g115(.a(new_n196), .b(new_n203), .c(new_n206), .d(new_n202), .out0(new_n211));
  tech160nm_fioai012aa1n03p5x5 g116(.a(new_n205), .b(new_n204), .c(new_n202), .o1(new_n212));
  nona23aa1n09x5               g117(.a(new_n205), .b(new_n203), .c(new_n202), .d(new_n204), .out0(new_n213));
  oai012aa1n18x5               g118(.a(new_n212), .b(new_n213), .c(new_n198), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n211), .c(new_n180), .d(new_n185), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n03x4               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  nand42aa1n03x5               g130(.a(new_n225), .b(new_n224), .o1(new_n226));
  tech160nm_finand02aa1n03p5x5 g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  oai112aa1n02x5               g132(.a(new_n226), .b(new_n227), .c(\b[20] ), .d(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n219), .b(new_n228), .out0(new_n229));
  nanb02aa1n02x5               g134(.a(new_n211), .b(new_n229), .out0(new_n230));
  aoi022aa1n02x5               g135(.a(new_n214), .b(new_n229), .c(new_n227), .d(new_n228), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n180), .d(new_n185), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n04x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  nand42aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  orn002aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .o(new_n236));
  nanp02aa1n02x5               g141(.a(\b[23] ), .b(\a[24] ), .o1(new_n237));
  aoi122aa1n06x5               g142(.a(new_n234), .b(new_n236), .c(new_n237), .d(new_n232), .e(new_n235), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n234), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n235), .b(new_n234), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n232), .b(new_n240), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n236), .b(new_n237), .o1(new_n242));
  tech160nm_fiaoi012aa1n02p5x5 g147(.a(new_n242), .b(new_n241), .c(new_n239), .o1(new_n243));
  nor002aa1n02x5               g148(.a(new_n243), .b(new_n238), .o1(\s[24] ));
  nanp03aa1n02x5               g149(.a(new_n236), .b(new_n235), .c(new_n237), .o1(new_n245));
  nano23aa1n06x5               g150(.a(new_n245), .b(new_n228), .c(new_n219), .d(new_n239), .out0(new_n246));
  nanb02aa1n02x5               g151(.a(new_n211), .b(new_n246), .out0(new_n247));
  oaoi03aa1n02x5               g152(.a(new_n224), .b(new_n225), .c(new_n218), .o1(new_n248));
  nano23aa1n03x7               g153(.a(new_n248), .b(new_n242), .c(new_n239), .d(new_n235), .out0(new_n249));
  oaoi03aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .o1(new_n250));
  aoi112aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n214), .d(new_n246), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n247), .c(new_n180), .d(new_n185), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  aoi112aa1n02x5               g161(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n258));
  norb02aa1n03x4               g163(.a(new_n258), .b(new_n257), .out0(\s[26] ));
  inv000aa1d42x5               g164(.a(new_n109), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n120), .b(new_n125), .o1(new_n261));
  nand02aa1n02x5               g166(.a(new_n261), .b(new_n260), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n151), .b(new_n182), .o1(new_n263));
  nona22aa1n02x4               g168(.a(new_n263), .b(new_n184), .c(new_n183), .out0(new_n264));
  nano22aa1n03x7               g169(.a(new_n254), .b(new_n256), .c(new_n255), .out0(new_n265));
  nano22aa1n03x7               g170(.a(new_n211), .b(new_n265), .c(new_n246), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .d(new_n179), .o1(new_n267));
  nand22aa1n03x5               g172(.a(new_n214), .b(new_n246), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n249), .c(new_n250), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\a[25] ), .o1(new_n271));
  oaib12aa1n02x5               g176(.a(new_n256), .b(\b[24] ), .c(new_n271), .out0(new_n272));
  aoi022aa1n09x5               g177(.a(new_n269), .b(new_n265), .c(new_n270), .d(new_n272), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n267), .c(new_n273), .out0(\s[27] ));
  orn002aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .o(new_n276));
  aobi12aa1n02x7               g181(.a(new_n274), .b(new_n267), .c(new_n273), .out0(new_n277));
  norp02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .o1(new_n279));
  nanb02aa1n02x5               g184(.a(new_n278), .b(new_n279), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n277), .b(new_n276), .c(new_n280), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n265), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(new_n272), .b(new_n270), .o1(new_n283));
  tech160nm_fioai012aa1n05x5   g188(.a(new_n283), .b(new_n251), .c(new_n282), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n274), .b(new_n284), .c(new_n186), .d(new_n266), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n280), .b(new_n285), .c(new_n276), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n281), .o1(\s[28] ));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nanp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  nano22aa1n02x4               g194(.a(new_n280), .b(new_n276), .c(new_n289), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n284), .c(new_n186), .d(new_n266), .o1(new_n291));
  oaib12aa1n02x5               g196(.a(new_n279), .b(new_n278), .c(new_n276), .out0(new_n292));
  aoi012aa1n03x5               g197(.a(new_n288), .b(new_n291), .c(new_n292), .o1(new_n293));
  aobi12aa1n02x7               g198(.a(new_n290), .b(new_n267), .c(new_n273), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n288), .c(new_n292), .out0(new_n295));
  norp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n298));
  nano23aa1n02x4               g203(.a(new_n288), .b(new_n278), .c(new_n274), .d(new_n279), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n284), .c(new_n186), .d(new_n266), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  aoi012aa1n02x7               g206(.a(new_n301), .b(new_n300), .c(new_n298), .o1(new_n302));
  aobi12aa1n02x7               g207(.a(new_n299), .b(new_n267), .c(new_n273), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n298), .c(new_n301), .out0(new_n304));
  norp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  norb03aa1n02x5               g210(.a(new_n290), .b(new_n288), .c(new_n301), .out0(new_n306));
  aobi12aa1n02x7               g211(.a(new_n306), .b(new_n267), .c(new_n273), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n306), .b(new_n284), .c(new_n186), .d(new_n266), .o1(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n309), .b(new_n311), .c(new_n308), .o1(new_n312));
  nor002aa1n02x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiaoi012aa1n05x5   g222(.a(new_n107), .b(new_n120), .c(new_n122), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n03x5               g226(.a(new_n104), .b(new_n320), .out0(new_n322));
  xobna2aa1n03x5               g227(.a(new_n101), .b(new_n322), .c(new_n102), .out0(\s[8] ));
  xnbna2aa1n03x5               g228(.a(new_n127), .b(new_n261), .c(new_n260), .out0(\s[9] ));
endmodule


