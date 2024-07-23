// Benchmark "adder" written by ABC on Wed Jul 17 23:41:43 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n313, new_n315, new_n317, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv040aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  tech160nm_fixnrc02aa1n02p5x5 g006(.a(\b[2] ), .b(\a[3] ), .out0(new_n102));
  inv040aa1d32x5               g007(.a(\a[2] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[1] ), .o1(new_n104));
  nand02aa1d06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oao003aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .carry(new_n106));
  nanb02aa1n02x5               g011(.a(new_n102), .b(new_n106), .out0(new_n107));
  oa0022aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  and002aa1n18x5               g013(.a(\b[4] ), .b(\a[5] ), .o(new_n109));
  nand42aa1d28x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norb02aa1n03x5               g016(.a(new_n110), .b(new_n111), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  oai112aa1n02x5               g019(.a(new_n113), .b(new_n114), .c(\b[4] ), .d(\a[5] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand42aa1n16x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor022aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  norb03aa1n03x4               g023(.a(new_n117), .b(new_n116), .c(new_n118), .out0(new_n119));
  nona23aa1n03x5               g024(.a(new_n112), .b(new_n119), .c(new_n115), .d(new_n109), .out0(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  nanp03aa1n02x5               g026(.a(new_n121), .b(new_n114), .c(new_n117), .o1(new_n122));
  nona22aa1n02x4               g027(.a(new_n122), .b(new_n118), .c(new_n111), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n110), .o1(new_n124));
  aoai13aa1n04x5               g029(.a(new_n124), .b(new_n120), .c(new_n107), .d(new_n108), .o1(new_n125));
  nanp02aa1n24x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n100), .b(new_n101), .c(new_n125), .d(new_n126), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n128));
  oai012aa1n12x5               g033(.a(new_n108), .b(new_n128), .c(new_n102), .o1(new_n129));
  inv000aa1d42x5               g034(.a(new_n109), .o1(new_n130));
  nano32aa1n03x7               g035(.a(new_n115), .b(new_n119), .c(new_n130), .d(new_n112), .out0(new_n131));
  aoi113aa1n02x5               g036(.a(new_n118), .b(new_n111), .c(new_n121), .d(new_n117), .e(new_n114), .o1(new_n132));
  norb02aa1n06x4               g037(.a(new_n110), .b(new_n132), .out0(new_n133));
  norb02aa1n02x5               g038(.a(new_n126), .b(new_n101), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n135));
  nona22aa1n02x4               g040(.a(new_n135), .b(new_n101), .c(new_n100), .out0(new_n136));
  nanp02aa1n02x5               g041(.a(new_n127), .b(new_n136), .o1(\s[10] ));
  nor002aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n06x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xobna2aa1n03x5               g045(.a(new_n140), .b(new_n136), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n02x4               g046(.a(new_n138), .b(new_n136), .c(new_n98), .d(new_n140), .o1(new_n142));
  nor042aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n06x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(new_n142), .b(new_n145), .out0(\s[12] ));
  nano23aa1n06x5               g051(.a(new_n97), .b(new_n101), .c(new_n126), .d(new_n98), .out0(new_n147));
  nand23aa1n09x5               g052(.a(new_n147), .b(new_n140), .c(new_n145), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n150));
  oai112aa1n02x5               g055(.a(new_n139), .b(new_n98), .c(new_n101), .d(new_n97), .o1(new_n151));
  nona22aa1n02x4               g056(.a(new_n151), .b(new_n143), .c(new_n138), .out0(new_n152));
  nanp02aa1n03x5               g057(.a(new_n152), .b(new_n144), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n150), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[13] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(\b[12] ), .o1(new_n157));
  oaoi03aa1n02x5               g062(.a(new_n156), .b(new_n157), .c(new_n154), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nor022aa1n04x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand42aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n03x5               g068(.a(new_n163), .b(new_n161), .c(new_n160), .d(new_n162), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n163), .b(new_n162), .c(new_n156), .d(new_n157), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n164), .c(new_n150), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xnrc02aa1n12x5               g073(.a(\b[14] ), .b(\a[15] ), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  xnrc02aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n168), .b(new_n172), .c(new_n166), .d(new_n170), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n170), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n02x5               g080(.a(new_n160), .b(new_n162), .c(new_n163), .d(new_n161), .out0(new_n176));
  nona22aa1n03x5               g081(.a(new_n176), .b(new_n169), .c(new_n171), .out0(new_n177));
  nor042aa1n06x5               g082(.a(new_n177), .b(new_n148), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n179));
  nor043aa1n02x5               g084(.a(new_n164), .b(new_n169), .c(new_n171), .o1(new_n180));
  oaoi13aa1n02x5               g085(.a(new_n168), .b(new_n163), .c(new_n160), .d(new_n162), .o1(new_n181));
  ao0022aa1n03x5               g086(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n182));
  oai022aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  aoi013aa1n06x4               g088(.a(new_n183), .b(new_n180), .c(new_n152), .d(new_n144), .o1(new_n184));
  nanp02aa1n06x5               g089(.a(new_n179), .b(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  oai022aa1d24x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  oaib12aa1n18x5               g098(.a(new_n193), .b(new_n187), .c(\b[17] ), .out0(new_n194));
  inv000aa1n12x5               g099(.a(new_n194), .o1(new_n195));
  nor042aa1d18x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n24x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n200));
  norb02aa1n02x7               g105(.a(new_n199), .b(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand02aa1n16x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nona22aa1n02x5               g110(.a(new_n199), .b(new_n205), .c(new_n196), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n205), .o1(new_n207));
  oaoi13aa1n06x5               g112(.a(new_n207), .b(new_n199), .c(\a[19] ), .d(\b[18] ), .o1(new_n208));
  norb02aa1n03x4               g113(.a(new_n206), .b(new_n208), .out0(\s[20] ));
  nano23aa1n06x5               g114(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n192), .b(new_n210), .o1(new_n211));
  nona23aa1n09x5               g116(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n212));
  aoi012aa1n06x5               g117(.a(new_n203), .b(new_n196), .c(new_n204), .o1(new_n213));
  oai012aa1n18x5               g118(.a(new_n213), .b(new_n212), .c(new_n194), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n04x5               g120(.a(new_n215), .b(new_n211), .c(new_n179), .d(new_n184), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x7               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv040aa1d32x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n192), .c(new_n210), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[21] ), .o1(new_n228));
  tech160nm_fioaoi03aa1n03p5x5 g133(.a(new_n225), .b(new_n228), .c(new_n218), .o1(new_n229));
  inv000aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n214), .c(new_n226), .o1(new_n231));
  aoai13aa1n04x5               g136(.a(new_n231), .b(new_n227), .c(new_n179), .d(new_n184), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n12x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n238));
  norb02aa1n02x7               g143(.a(new_n238), .b(new_n237), .out0(\s[24] ));
  oabi12aa1n06x5               g144(.a(new_n183), .b(new_n153), .c(new_n177), .out0(new_n240));
  and002aa1n02x5               g145(.a(new_n236), .b(new_n235), .o(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n226), .c(new_n192), .d(new_n210), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n125), .d(new_n178), .o1(new_n244));
  inv020aa1n02x5               g149(.a(new_n213), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n226), .b(new_n245), .c(new_n210), .d(new_n195), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n247));
  oab012aa1n02x4               g152(.a(new_n247), .b(\a[24] ), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n12x5               g153(.a(new_n248), .b(new_n242), .c(new_n246), .d(new_n229), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[24] ), .b(\a[25] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n244), .c(new_n250), .out0(\s[25] ));
  nor042aa1n03x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n252), .b(new_n249), .c(new_n185), .d(new_n243), .o1(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[25] ), .b(\a[26] ), .out0(new_n257));
  nanp03aa1n03x5               g162(.a(new_n256), .b(new_n255), .c(new_n257), .o1(new_n258));
  tech160nm_fiaoi012aa1n02p5x5 g163(.a(new_n257), .b(new_n256), .c(new_n255), .o1(new_n259));
  norb02aa1n03x4               g164(.a(new_n258), .b(new_n259), .out0(\s[26] ));
  nor042aa1n04x5               g165(.a(new_n257), .b(new_n251), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n227), .b(new_n241), .c(new_n261), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n240), .c(new_n125), .d(new_n178), .o1(new_n263));
  nand02aa1d10x5               g168(.a(new_n249), .b(new_n261), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n265));
  xorc02aa1n12x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoi013aa1n06x4               g172(.a(new_n267), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n262), .o1(new_n269));
  aoi012aa1n06x5               g174(.a(new_n269), .b(new_n179), .c(new_n184), .o1(new_n270));
  aoai13aa1n02x7               g175(.a(new_n241), .b(new_n230), .c(new_n214), .d(new_n226), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n261), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n265), .b(new_n272), .c(new_n271), .d(new_n248), .o1(new_n273));
  norp03aa1n02x5               g178(.a(new_n273), .b(new_n270), .c(new_n266), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n268), .b(new_n274), .o1(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  xnrc02aa1n12x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n268), .b(new_n277), .c(new_n278), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n266), .b(new_n273), .c(new_n270), .o1(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n278), .b(new_n280), .c(new_n277), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n281), .b(new_n279), .o1(\s[28] ));
  norb02aa1n12x5               g187(.a(new_n266), .b(new_n278), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n273), .c(new_n270), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  aoi012aa1n03x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n283), .o1(new_n288));
  aoi013aa1n02x5               g193(.a(new_n288), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n285), .c(new_n286), .out0(new_n290));
  nor002aa1n02x5               g195(.a(new_n287), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g197(.a(new_n266), .b(new_n286), .c(new_n278), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n273), .c(new_n270), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  aoi012aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n293), .o1(new_n298));
  aoi013aa1n03x5               g203(.a(new_n298), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n296), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n297), .b(new_n300), .o1(\s[30] ));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n293), .b(new_n296), .out0(new_n303));
  inv000aa1n02x5               g208(.a(new_n303), .o1(new_n304));
  aoi013aa1n03x5               g209(.a(new_n304), .b(new_n263), .c(new_n264), .d(new_n265), .o1(new_n305));
  oao003aa1n03x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n306));
  nano22aa1n03x5               g211(.a(new_n305), .b(new_n302), .c(new_n306), .out0(new_n307));
  oaih12aa1n02x5               g212(.a(new_n303), .b(new_n273), .c(new_n270), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n302), .b(new_n308), .c(new_n306), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g215(.a(new_n128), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  xnrc02aa1n02x5               g216(.a(\b[3] ), .b(\a[4] ), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n128), .carry(new_n313));
  mtn022aa1n02x5               g218(.a(new_n313), .b(new_n129), .sa(new_n312), .o1(\s[4] ));
  xnrc02aa1n02x5               g219(.a(\b[4] ), .b(\a[5] ), .out0(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n315), .b(new_n129), .c(new_n113), .out0(\s[5] ));
  norp02aa1n02x5               g221(.a(\b[4] ), .b(\a[5] ), .o1(new_n317));
  aoi013aa1n02x4               g222(.a(new_n317), .b(new_n129), .c(new_n130), .d(new_n113), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n118), .b(new_n320), .c(new_n117), .o1(new_n322));
  xnrc02aa1n02x5               g227(.a(new_n322), .b(new_n112), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


