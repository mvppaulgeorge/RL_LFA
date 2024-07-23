// Benchmark "adder" written by ABC on Thu Jul 18 03:14:26 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n312, new_n314, new_n315, new_n318, new_n319,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand42aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oaoi03aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .out0(new_n103));
  orn002aa1n02x5               g008(.a(\a[4] ), .b(\b[3] ), .o(new_n104));
  aoi112aa1n02x7               g009(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  oai013aa1n03x5               g011(.a(new_n106), .b(new_n101), .c(new_n102), .d(new_n103), .o1(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .out0(new_n108));
  tech160nm_fixorc02aa1n03p5x5 g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  xorc02aa1n12x5               g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nanp02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1d18x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanb02aa1n06x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nano23aa1n02x5               g018(.a(new_n113), .b(new_n108), .c(new_n109), .d(new_n110), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nano22aa1n03x7               g020(.a(new_n112), .b(new_n115), .c(new_n111), .out0(new_n116));
  oai022aa1n06x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n112), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n119));
  aoi013aa1n09x5               g024(.a(new_n119), .b(new_n116), .c(new_n110), .d(new_n117), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n120), .o1(new_n121));
  xnrc02aa1n12x5               g026(.a(\b[8] ), .b(\a[9] ), .out0(new_n122));
  inv000aa1d42x5               g027(.a(new_n122), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n121), .c(new_n114), .d(new_n107), .o1(new_n124));
  xorc02aa1n06x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  oai022aa1n09x5               g031(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n127));
  nanb02aa1n02x5               g032(.a(new_n127), .b(new_n124), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nor002aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb03aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n130), .out0(new_n132));
  nanb02aa1n06x5               g037(.a(new_n131), .b(new_n130), .out0(new_n133));
  oao003aa1n02x5               g038(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n134));
  xorc02aa1n02x5               g039(.a(\a[4] ), .b(\b[3] ), .out0(new_n135));
  xorc02aa1n02x5               g040(.a(\a[3] ), .b(\b[2] ), .out0(new_n136));
  nanp03aa1n02x5               g041(.a(new_n134), .b(new_n135), .c(new_n136), .o1(new_n137));
  nona23aa1n03x5               g042(.a(new_n109), .b(new_n110), .c(new_n108), .d(new_n113), .out0(new_n138));
  aoai13aa1n12x5               g043(.a(new_n120), .b(new_n138), .c(new_n137), .d(new_n106), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n129), .b(new_n127), .c(new_n139), .d(new_n123), .o1(new_n140));
  aboi22aa1n03x5               g045(.a(new_n132), .b(new_n128), .c(new_n140), .d(new_n133), .out0(\s[11] ));
  xorc02aa1n06x5               g046(.a(\a[12] ), .b(\b[11] ), .out0(new_n142));
  aoi113aa1n02x5               g047(.a(new_n142), .b(new_n131), .c(new_n128), .d(new_n130), .e(new_n129), .o1(new_n143));
  obai22aa1n02x7               g048(.a(new_n128), .b(new_n132), .c(\a[11] ), .d(\b[10] ), .out0(new_n144));
  aoi012aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n142), .o1(\s[12] ));
  nona23aa1d18x5               g050(.a(new_n142), .b(new_n125), .c(new_n122), .d(new_n133), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoi022aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n148));
  aoai13aa1n03x5               g053(.a(new_n148), .b(new_n131), .c(new_n127), .d(new_n129), .o1(new_n149));
  tech160nm_fioai012aa1n04x5   g054(.a(new_n149), .b(\b[11] ), .c(\a[12] ), .o1(new_n150));
  xorc02aa1n02x5               g055(.a(\a[13] ), .b(\b[12] ), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n150), .c(new_n139), .d(new_n147), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(new_n151), .b(new_n150), .c(new_n139), .d(new_n147), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(\b[12] ), .o1(new_n156));
  nand42aa1n03x5               g061(.a(new_n156), .b(new_n155), .o1(new_n157));
  nor022aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1n03x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n152), .c(new_n157), .out0(\s[14] ));
  nand42aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nano32aa1n03x7               g067(.a(new_n158), .b(new_n157), .c(new_n159), .d(new_n162), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n150), .c(new_n139), .d(new_n147), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n159), .b(new_n158), .c(new_n155), .d(new_n156), .o1(new_n165));
  xorc02aa1n02x5               g070(.a(\a[15] ), .b(\b[14] ), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n164), .c(new_n165), .out0(\s[15] ));
  aob012aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n165), .out0(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  orn002aa1n02x5               g076(.a(\a[15] ), .b(\b[14] ), .o(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  nand02aa1d06x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n172), .b(new_n175), .c(new_n164), .d(new_n165), .o1(new_n176));
  aoi022aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n168), .d(new_n173), .o1(\s[16] ));
  nano32aa1n02x4               g082(.a(new_n169), .b(new_n172), .c(new_n170), .d(new_n174), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n178), .b(new_n163), .o1(new_n179));
  nor042aa1n04x5               g084(.a(new_n146), .b(new_n179), .o1(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n121), .c(new_n114), .d(new_n107), .o1(new_n181));
  oai122aa1n02x7               g086(.a(new_n172), .b(new_n165), .c(new_n175), .d(\b[15] ), .e(\a[16] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n170), .o1(new_n183));
  oaib12aa1n06x5               g088(.a(new_n183), .b(new_n179), .c(new_n150), .out0(new_n184));
  nanb02aa1n09x5               g089(.a(new_n184), .b(new_n181), .out0(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(\b[16] ), .b(new_n187), .out0(new_n188));
  xorc02aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n190));
  xorc02aa1n02x5               g095(.a(\a[18] ), .b(\b[17] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n188), .out0(\s[18] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n187), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n03x5               g099(.a(new_n194), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n195));
  oai022aa1n02x7               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n12x5               g101(.a(new_n196), .b(new_n193), .c(\b[17] ), .out0(new_n197));
  nor042aa1d18x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand02aa1d04x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g108(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n201), .b(new_n204), .c(new_n185), .d(new_n194), .o1(new_n205));
  nor042aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1d06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoib12aa1n02x5               g113(.a(new_n198), .b(new_n207), .c(new_n206), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n198), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n200), .c(new_n195), .d(new_n197), .o1(new_n211));
  aoi022aa1n03x5               g116(.a(new_n211), .b(new_n208), .c(new_n205), .d(new_n209), .o1(\s[20] ));
  nona23aa1d18x5               g117(.a(new_n207), .b(new_n199), .c(new_n198), .d(new_n206), .out0(new_n213));
  nano22aa1n02x4               g118(.a(new_n213), .b(new_n189), .c(new_n191), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n215));
  aoi012aa1n09x5               g120(.a(new_n206), .b(new_n198), .c(new_n207), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n213), .c(new_n197), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[20] ), .b(\a[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n215), .c(new_n218), .out0(\s[21] ));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n217), .c(new_n185), .d(new_n214), .o1(new_n222));
  xnrc02aa1n03x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n224), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n227));
  aboi22aa1n03x5               g132(.a(new_n223), .b(new_n227), .c(new_n222), .d(new_n225), .out0(\s[22] ));
  nano23aa1n06x5               g133(.a(new_n198), .b(new_n206), .c(new_n207), .d(new_n199), .out0(new_n229));
  nor042aa1n06x5               g134(.a(new_n223), .b(new_n219), .o1(new_n230));
  and003aa1n02x5               g135(.a(new_n194), .b(new_n230), .c(new_n229), .o(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n232));
  oao003aa1n12x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .carry(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1d18x5               g139(.a(new_n234), .b(new_n217), .c(new_n230), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[22] ), .b(\a[23] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n235), .out0(\s[23] ));
  inv000aa1d42x5               g143(.a(new_n235), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n185), .d(new_n231), .o1(new_n240));
  tech160nm_fixorc02aa1n02p5x5 g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  nor042aa1n03x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  norp02aa1n02x5               g147(.a(new_n241), .b(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n242), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n245));
  aoi022aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n240), .d(new_n243), .o1(\s[24] ));
  norb02aa1n02x7               g151(.a(new_n241), .b(new_n236), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  nano32aa1n02x4               g153(.a(new_n248), .b(new_n194), .c(new_n230), .d(new_n229), .out0(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n250));
  inv020aa1n03x5               g155(.a(new_n216), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n230), .b(new_n251), .c(new_n229), .d(new_n204), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n253));
  aoai13aa1n12x5               g158(.a(new_n253), .b(new_n248), .c(new_n252), .d(new_n233), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xnbna2aa1n03x5               g161(.a(new_n256), .b(new_n250), .c(new_n255), .out0(\s[25] ));
  aoai13aa1n02x5               g162(.a(new_n256), .b(new_n254), .c(new_n185), .d(new_n249), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n259), .b(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n260), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n256), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n262), .b(new_n263), .c(new_n250), .d(new_n255), .o1(new_n264));
  aoi022aa1n03x5               g169(.a(new_n264), .b(new_n259), .c(new_n258), .d(new_n261), .o1(\s[26] ));
  and002aa1n02x5               g170(.a(new_n259), .b(new_n256), .o(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  nano32aa1n03x7               g172(.a(new_n267), .b(new_n214), .c(new_n230), .d(new_n247), .out0(new_n268));
  aoai13aa1n09x5               g173(.a(new_n268), .b(new_n184), .c(new_n139), .d(new_n180), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoi012aa1n12x5               g176(.a(new_n271), .b(new_n254), .c(new_n266), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n269), .out0(\s[27] ));
  aoai13aa1n06x5               g179(.a(new_n247), .b(new_n234), .c(new_n217), .d(new_n230), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n270), .b(new_n267), .c(new_n275), .d(new_n253), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n273), .b(new_n276), .c(new_n185), .d(new_n268), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .out0(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n278), .b(new_n279), .o1(new_n280));
  inv000aa1n03x5               g185(.a(new_n279), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n273), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n281), .b(new_n282), .c(new_n272), .d(new_n269), .o1(new_n283));
  aoi022aa1n02x5               g188(.a(new_n283), .b(new_n278), .c(new_n277), .d(new_n280), .o1(\s[28] ));
  and002aa1n02x5               g189(.a(new_n278), .b(new_n273), .o(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n276), .c(new_n185), .d(new_n268), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n285), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n287), .c(new_n272), .d(new_n269), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .out0(new_n290));
  norb02aa1n02x5               g195(.a(new_n288), .b(new_n290), .out0(new_n291));
  aoi022aa1n02x5               g196(.a(new_n289), .b(new_n290), .c(new_n286), .d(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g198(.a(new_n282), .b(new_n278), .c(new_n290), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n276), .c(new_n185), .d(new_n268), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n294), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n272), .d(new_n269), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .out0(new_n299));
  norb02aa1n02x5               g204(.a(new_n297), .b(new_n299), .out0(new_n300));
  aoi022aa1n03x5               g205(.a(new_n298), .b(new_n299), .c(new_n295), .d(new_n300), .o1(\s[30] ));
  nano32aa1n06x5               g206(.a(new_n282), .b(new_n299), .c(new_n278), .d(new_n290), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n276), .c(new_n185), .d(new_n268), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[31] ), .b(\b[30] ), .out0(new_n304));
  and002aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .o(new_n305));
  oabi12aa1n02x5               g210(.a(new_n304), .b(\a[30] ), .c(\b[29] ), .out0(new_n306));
  oab012aa1n02x4               g211(.a(new_n306), .b(new_n297), .c(new_n305), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n302), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n272), .d(new_n269), .o1(new_n310));
  aoi022aa1n02x5               g215(.a(new_n310), .b(new_n304), .c(new_n303), .d(new_n307), .o1(\s[31] ));
  inv000aa1d42x5               g216(.a(\a[3] ), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n101), .b(\b[2] ), .c(new_n312), .out0(\s[3] ));
  nanp02aa1n02x5               g218(.a(new_n134), .b(new_n136), .o1(new_n314));
  aoib12aa1n02x5               g219(.a(new_n135), .b(new_n312), .c(\b[2] ), .out0(new_n315));
  aoi022aa1n02x5               g220(.a(new_n107), .b(new_n104), .c(new_n315), .d(new_n314), .o1(\s[4] ));
  xnbna2aa1n03x5               g221(.a(new_n109), .b(new_n137), .c(new_n106), .out0(\s[5] ));
  norp02aa1n02x5               g222(.a(\b[4] ), .b(\a[5] ), .o1(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n107), .c(new_n109), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g225(.a(new_n108), .b(new_n319), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n113), .b(new_n321), .c(new_n115), .out0(\s[7] ));
  nanp02aa1n02x5               g227(.a(new_n321), .b(new_n116), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n110), .b(new_n323), .c(new_n118), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n139), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


