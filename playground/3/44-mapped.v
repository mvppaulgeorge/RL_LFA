// Benchmark "adder" written by ABC on Wed Jul 17 13:51:45 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv020aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  nand42aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nor042aa1n06x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand42aa1d28x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fioai012aa1n05x5   g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  oa0022aa1n02x5               g010(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n106));
  oaoi13aa1n12x5               g011(.a(new_n100), .b(new_n106), .c(new_n101), .d(new_n105), .o1(new_n107));
  nor022aa1n12x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand02aa1d04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand22aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n06x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  norb03aa1n03x5               g019(.a(new_n114), .b(new_n112), .c(new_n113), .out0(new_n115));
  aoi112aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n116));
  nano23aa1n06x5               g021(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n117));
  and002aa1n24x5               g022(.a(\b[5] ), .b(\a[6] ), .o(new_n118));
  inv000aa1d42x5               g023(.a(\a[5] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv040aa1d32x5               g025(.a(\b[4] ), .o1(new_n121));
  aboi22aa1d24x5               g026(.a(\b[5] ), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n122));
  nona22aa1n03x5               g027(.a(new_n117), .b(new_n118), .c(new_n122), .out0(new_n123));
  nona22aa1n06x5               g028(.a(new_n123), .b(new_n116), .c(new_n108), .out0(new_n124));
  tech160nm_fixorc02aa1n02p5x5 g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n107), .d(new_n115), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n97), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(new_n97), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n129));
  inv000aa1n02x5               g034(.a(new_n129), .o1(new_n130));
  aoai13aa1n03x5               g035(.a(new_n130), .b(new_n128), .c(new_n126), .d(new_n99), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n09x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1d24x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor042aa1n09x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand22aa1n09x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n06x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nano23aa1n03x7               g045(.a(new_n133), .b(new_n135), .c(new_n136), .d(new_n134), .out0(new_n141));
  nand02aa1d04x5               g046(.a(new_n125), .b(new_n97), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n124), .c(new_n107), .d(new_n115), .o1(new_n144));
  aoi112aa1n09x5               g049(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n145));
  nor042aa1n02x5               g050(.a(\b[9] ), .b(\a[10] ), .o1(new_n146));
  norb02aa1n06x5               g051(.a(new_n134), .b(new_n133), .out0(new_n147));
  aoi112aa1n09x5               g052(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n148));
  oai112aa1n06x5               g053(.a(new_n137), .b(new_n147), .c(new_n148), .d(new_n146), .o1(new_n149));
  nona22aa1d18x5               g054(.a(new_n149), .b(new_n145), .c(new_n135), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n144), .b(new_n151), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n03x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n152), .c(new_n155), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand22aa1n03x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n159), .b(new_n155), .c(new_n154), .d(new_n158), .out0(new_n160));
  aoi012aa1n02x7               g065(.a(new_n158), .b(new_n154), .c(new_n159), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n160), .c(new_n144), .d(new_n151), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor002aa1d32x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nand42aa1n06x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanb02aa1n18x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nano23aa1n02x5               g077(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n173));
  nano23aa1n06x5               g078(.a(new_n160), .b(new_n142), .c(new_n173), .d(new_n141), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n124), .c(new_n107), .d(new_n115), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n164), .b(new_n165), .out0(new_n176));
  nor003aa1n03x5               g081(.a(new_n160), .b(new_n168), .c(new_n176), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n166), .o1(new_n179));
  oai013aa1n03x5               g084(.a(new_n179), .b(new_n161), .c(new_n176), .d(new_n168), .o1(new_n180));
  aoi112aa1n09x5               g085(.a(new_n180), .b(new_n178), .c(new_n150), .d(new_n177), .o1(new_n181));
  nand02aa1d08x5               g086(.a(new_n175), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g088(.a(\a[18] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  oaoi03aa1n03x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nand02aa1d04x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nona22aa1n02x4               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(new_n191));
  oaib12aa1n09x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n184), .out0(new_n192));
  nor022aa1n08x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand42aa1n06x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand42aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x5               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  orn002aa1n24x5               g108(.a(\a[19] ), .b(\b[18] ), .o(new_n204));
  aobi12aa1n02x7               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n03x7               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n189), .b(new_n207), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  aoi013aa1n06x4               g114(.a(new_n209), .b(new_n190), .c(new_n185), .d(new_n186), .o1(new_n210));
  nona23aa1n12x5               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  oaoi03aa1n12x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n212));
  inv030aa1n02x5               g117(.a(new_n212), .o1(new_n213));
  oai012aa1n12x5               g118(.a(new_n213), .b(new_n211), .c(new_n210), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n208), .c(new_n175), .d(new_n181), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n189), .c(new_n207), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[21] ), .o1(new_n228));
  oao003aa1n06x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n226), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n227), .c(new_n175), .d(new_n181), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  norp02aa1n02x5               g139(.a(\b[23] ), .b(\a[24] ), .o1(new_n235));
  nand42aa1n02x5               g140(.a(\b[23] ), .b(\a[24] ), .o1(new_n236));
  norb02aa1n06x4               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n233), .b(new_n237), .c(new_n231), .d(new_n234), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n237), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  and002aa1n06x5               g145(.a(new_n234), .b(new_n237), .o(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n226), .c(new_n189), .d(new_n207), .out0(new_n243));
  aoai13aa1n03x5               g148(.a(new_n226), .b(new_n212), .c(new_n207), .d(new_n192), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n229), .o1(new_n245));
  oai012aa1n02x5               g150(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n244), .d(new_n245), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n247), .c(new_n182), .d(new_n243), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n182), .d(new_n243), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  nor042aa1n03x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  nona22aa1n02x5               g158(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n252), .o1(new_n255));
  aobi12aa1n02x7               g160(.a(new_n253), .b(new_n249), .c(new_n255), .out0(new_n256));
  norb02aa1n03x4               g161(.a(new_n254), .b(new_n256), .out0(\s[26] ));
  oai012aa1n02x5               g162(.a(new_n106), .b(new_n101), .c(new_n105), .o1(new_n258));
  nanb02aa1n02x5               g163(.a(new_n100), .b(new_n258), .out0(new_n259));
  nanb03aa1n02x5               g164(.a(new_n113), .b(new_n117), .c(new_n114), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n118), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n122), .o1(new_n262));
  aoi113aa1n02x5               g167(.a(new_n108), .b(new_n116), .c(new_n117), .d(new_n261), .e(new_n262), .o1(new_n263));
  oai012aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n260), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n150), .b(new_n177), .o1(new_n265));
  nona22aa1n03x5               g170(.a(new_n265), .b(new_n180), .c(new_n178), .out0(new_n266));
  inv000aa1d42x5               g171(.a(\a[25] ), .o1(new_n267));
  inv020aa1n04x5               g172(.a(\a[26] ), .o1(new_n268));
  xroi22aa1d06x4               g173(.a(new_n267), .b(\b[24] ), .c(new_n268), .d(\b[25] ), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n227), .b(new_n241), .c(new_n269), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n266), .c(new_n264), .d(new_n174), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n272));
  aobi12aa1n06x5               g177(.a(new_n272), .b(new_n247), .c(new_n269), .out0(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aobi12aa1n02x7               g182(.a(new_n274), .b(new_n273), .c(new_n271), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n03x5               g184(.a(new_n278), .b(new_n277), .c(new_n279), .out0(new_n280));
  inv020aa1n03x5               g185(.a(new_n270), .o1(new_n281));
  aoi012aa1n06x5               g186(.a(new_n281), .b(new_n175), .c(new_n181), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n241), .b(new_n229), .c(new_n214), .d(new_n226), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n269), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n272), .b(new_n284), .c(new_n283), .d(new_n246), .o1(new_n285));
  oaih12aa1n02x5               g190(.a(new_n274), .b(new_n285), .c(new_n282), .o1(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n279), .b(new_n286), .c(new_n277), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g193(.a(new_n274), .b(new_n279), .out0(new_n289));
  aobi12aa1n02x7               g194(.a(new_n289), .b(new_n273), .c(new_n271), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n290), .b(new_n291), .c(new_n292), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n289), .b(new_n285), .c(new_n282), .o1(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n292), .b(new_n294), .c(new_n291), .o1(new_n295));
  norp02aa1n03x5               g200(.a(new_n295), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g202(.a(new_n274), .b(new_n292), .c(new_n279), .out0(new_n298));
  aobi12aa1n02x7               g203(.a(new_n298), .b(new_n273), .c(new_n271), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n298), .b(new_n285), .c(new_n282), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g210(.a(new_n298), .b(new_n301), .out0(new_n306));
  aobi12aa1n02x7               g211(.a(new_n306), .b(new_n273), .c(new_n271), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  oaih12aa1n02x5               g215(.a(new_n306), .b(new_n285), .c(new_n282), .o1(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n309), .b(new_n311), .c(new_n308), .o1(new_n312));
  norp02aa1n03x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g222(.a(new_n119), .b(new_n121), .c(new_n107), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n110), .b(new_n320), .c(new_n111), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n264), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


